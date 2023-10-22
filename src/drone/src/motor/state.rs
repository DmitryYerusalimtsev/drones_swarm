use std::f64::consts::PI;
use tokio::time::{sleep, Duration};

const AIR_DENSITY: f64 = 1.225; // kg/m³
const LIFT_COEFFICIENT: f64 = 1.1554; // Example
const NUMBER_OF_ITERATIONS: i32 = 4;

#[derive(Clone, Copy)]
pub struct State {
    blades_radius: f64, // square meters
    rotor_disk_area: f64,
    pub rpm: f64,
    pub thrust: f64
}

impl State {

    pub fn new(blades_radius: f64) -> Self {
        Self {
            blades_radius: blades_radius, 
            rotor_disk_area: PI * blades_radius.powi(2), 
            rpm: 0.0, 
            thrust: 0.0
         }
    }

    pub async fn set_thrust(mut self, required_thrust: f64) {
        let rpm = self.rpm_to_achieve_thrust(required_thrust);

        while self.thrust != required_thrust {
            self.thrust += required_thrust / NUMBER_OF_ITERATIONS as f64;
            self.rpm += rpm / NUMBER_OF_ITERATIONS as f64;

            sleep(Duration::from_millis(1000 / NUMBER_OF_ITERATIONS as u64)).await;
        }
    }

    fn rpm_to_achieve_thrust(self, required_thrust: f64) -> f64 {
        
        let lift_equation = 0.5 * LIFT_COEFFICIENT * self.rotor_disk_area * AIR_DENSITY * (2.0 * PI * self.blades_radius).powi(2);
        let rpm = (required_thrust / lift_equation).sqrt();

        rpm
    }
}
