use std::sync::{Arc, Mutex};
use std::time::Duration;
use r2r::{Node, QosProfile, Result};
use r2r::drone_msgs::srv::SetThrust;
use r2r::drone_msgs::msg::MotorState;
use tokio::task;
use futures::StreamExt;

use crate::state::State;

pub struct Motor {
    pub node: Arc<Mutex<Node>>,
    state: Arc<Mutex<State>>
}

impl Motor {
    pub fn new(node: Node, initial_state: State) -> Arc<Motor> {
        let motor = Arc::new(Self { 
            node: Arc::new(Mutex::new(node)), 
            state: Arc::new(Mutex::new(initial_state))
        });

        let pub_motor = Arc::clone(&motor);
        task::spawn(async move { pub_motor.start_state_publishing().await });

        let set_thrust_motor = Arc::clone(&motor);
        task::spawn(async move { set_thrust_motor.set_thrust().await });

        motor
    }

    async fn set_thrust(&self) {

        let mut service = {
            let mut node = self.node.lock().unwrap();
            let service_name = format!("{}/set_thrust", node.fully_qualified_name().unwrap());
            let service = node.create_service::<SetThrust::Service>(service_name.as_str());
            service.unwrap()
        };
    
        while let Some(request) = service.next().await {
            println!("Set thrust called.");

            let mut state = self.state.lock().unwrap();
            state.set_thrust(request.message.thrust);

            let response = SetThrust::Response {
                success: true,
                message: "".to_string()
            };
          
            request.respond(response).expect("could not send service response");
        }
    }
    
    async fn start_state_publishing(&self) -> Result<()> {
        let state_mtx = Arc::clone(&self.state);

        let (mut timer, publisher) = {
            let mut node = self.node.lock().unwrap();
            let timer = node.create_wall_timer(Duration::from_millis(500))?;
            let topic = format!("/{}/state", node.name()?);
            let publisher = node.create_publisher::<MotorState>(topic.as_str(), QosProfile::default())?;
            (timer, publisher)
        };

        loop {
            timer.tick().await?;
            {
                let state = *state_mtx.lock().unwrap();
                let message = MotorState {
                    rpm: state.rpm,
                    thrust: state.thrust
                };
                publisher.publish(&message)?;
            }
        }
    }
}