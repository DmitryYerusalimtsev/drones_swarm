mod drone;
mod state;

use std::collections::HashSet;
use std::time::Duration;
use r2r::{Node, Context};
use tokio::task;

use crate::drone::Drone;
use crate::state::State;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::create()?;
    let node = Node::create(ctx, "drone", "")?;

    println!("node name: {}", node.name()?);
    println!("node fully qualified name: {}", node.fully_qualified_name()?);

    let mut motors = HashSet::new();
    motors.insert(node.name()? + "/motor_0");
    // motors.insert(node.name()? + "/motor_1");
    // motors.insert(node.name()? + "/motor_2");
    // motors.insert(node.name()? + "/motor_3");

    let initial_state = State::new(1.5, motors);
    let drone = Drone::new(node, initial_state);
    
    task::spawn_blocking(move || loop {
        let mut node = drone.node.lock().unwrap();
        node.spin_once(Duration::from_millis(100));
    }).await?;

    Ok(())
}
