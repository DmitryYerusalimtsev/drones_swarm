use rust_3d::Point3D;
use std::collections::HashSet;

#[derive(Clone)]
pub struct State {
    pub weight: f64, // kg
    pub position: Option<Point3D>,
    pub motors: HashSet<String>
}

impl State {
    pub fn new(weight: f64, motors: HashSet<String>) -> Self {
        Self {
            weight,
            position: None,
            motors
        }
    }
}