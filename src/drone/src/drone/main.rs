mod drone;
mod state;

use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use r2r::{Node, Context, RosParams};
use tokio::task;

use crate::drone::Drone;
use crate::state::State;

#[derive(RosParams, Default, Debug)]
struct Params {
    motors: String
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "drone", "")?;

    let params = Arc::new(Mutex::new({
        let mut par = Params::default();
        par.motors = "motor_0,motor_1,motor_2,motor_3".into();
        par
    }));
    let (_, _) = node.make_derived_parameter_handler(params.clone())?;

    println!("Node started: {}", node.fully_qualified_name()?);
    println!("Params: {:#?}", params.clone().lock().unwrap());

    let mut motors = HashSet::new();
    params.lock().unwrap()
        .motors.split(",")
        .for_each(|motor| { motors.insert(node.name().unwrap() + "/" + motor); });

    let initial_state = State::new(1.5, motors);
    let drone = Drone::new(node, initial_state);
    
    task::spawn_blocking(move || loop {
        let mut node = drone.node.lock().unwrap();
        node.spin_once(Duration::from_millis(100));
    }).await?;

    Ok(())
}
