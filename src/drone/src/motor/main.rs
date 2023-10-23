mod motor;
mod state;

use std::time::Duration;
use r2r::{Node, Context};
use tokio::task;

use crate::motor::Motor;
use crate::state::State;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::create()?;
    let node = Node::create(ctx, "motor_0", "drone")?;

    println!("node name: {}", node.name()?);
    println!("node fully qualified name: {}", node.fully_qualified_name()?);

    let initial_state = State::new(0.05);
    let motor = Motor::new(node, initial_state);
    
    task::spawn_blocking(move || loop {
        let mut node = motor.node.lock().unwrap();
        node.spin_once(Duration::from_millis(100));
    }).await?;

    Ok(())
}