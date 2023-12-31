use std::collections::HashMap;
use std::sync::{Mutex, Arc};
use futures::StreamExt;
use r2r::drone_msgs::srv::SetThrust;
use r2r::{Node, Error, Client};
use r2r::std_srvs::srv::Trigger;
use tokio::task;

use crate::state::State;

pub struct Drone {
    pub node: Arc<Mutex<Node>>,
    state: Arc<Mutex<State>>,
    motor_clients: Arc<HashMap<String, Client<SetThrust::Service>>>,
}

impl Drone {
    pub fn new(mut node: Node, initial_state: State) -> Arc<Drone> {
        let motor_clients = initial_state.motors.iter().map(|motor| {
            let service = motor.clone() + "/set_thrust";
            let client = node.create_client::<SetThrust::Service>(service.as_str()).unwrap();
            (motor.clone(), client)
        }).collect();

        let drone = Arc::new(Self { 
            node: Arc::new(Mutex::new(node)), 
            state: Arc::new(Mutex::new(initial_state)),
            motor_clients: Arc::new(motor_clients),
        });

        let take_off_drone = Arc::clone(&drone);
        task::spawn(async move { take_off_drone.take_off().await });

        drone
    }

    async fn take_off(&self) {
        let node_mtx = Arc::clone(&self.node);
        let state_mtx = Arc::clone(&self.state);

        let mut service = {
            let mut node = node_mtx.lock().unwrap();
            node.create_service::<Trigger::Service>("/take_off").unwrap()
        };
    
        while let Some(request) = service.next().await {
            println!("Took off called.");

            let target_thrust = {
                let state = state_mtx.lock().unwrap();
                let thrust_per_motor = state.weight / state.motors.len() as f64;
                state.motors.iter().map(|m| (m.to_string(), thrust_per_motor)).collect()
            };

            let results = self.set_thrust(target_thrust).await;
            
            let error_message = results.iter()
                .find(|(_, r)| !r.as_ref().unwrap().success)
                .map( |(_, r)| r.as_ref().unwrap().message.clone())
                .or(None);

            let response = Trigger::Response {
                success: error_message.is_none(),
                message: error_message.or(Some("".to_string())).unwrap()
            };
          
            request.respond(response).expect("could not send service response");
        }
    }

    async fn set_thrust(&self, target_thrust: HashMap<String, f64>) 
        -> HashMap<String, Result<SetThrust::Response, Error>> {

        let tasks_number = target_thrust.len();

        futures::stream::iter(
            target_thrust
                .into_iter()
                .map(|(motor, thrust)| {
                    let clients = Arc::clone(&self.motor_clients);

                    task::spawn(async move {
                        let client = clients.get(&motor).unwrap();
                        let request = SetThrust::Request { thrust };
                        let response = client.request(&request).unwrap().await;
                        (motor, response)
                    })
                })
        )
        .buffer_unordered(tasks_number)
        .map(|res| res.unwrap())
        .collect::<HashMap<_, _>>()
        .await
    }
}