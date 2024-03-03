use nannou::prelude::*;

#[derive(Debug, Clone, Copy)]
pub enum Solver {
    Analytic(AnalyticTwoLink),
}

// Likely to move this to it's own file in case i implement more solvers
#[derive(Debug, Clone, Copy)]
pub struct AnalyticTwoLink {
    pub angle: f32,
    pub first_arm_length: f32,
    pub second_arm_length: f32,
    pub first_to_last_dist: f32,
    pub points: [Vec2; 3],

}

impl AnalyticTwoLink {
    pub fn new(angle: f32, f_len: f32, s_len: f32) -> AnalyticTwoLink {
        AnalyticTwoLink {
            angle: angle,
            first_arm_length: f_len, 
            second_arm_length: s_len, 
            first_to_last_dist: f_len + s_len, 
            points: [
                Vec2::ZERO, 
                Vec2::new(angle.cos() * f_len, angle.sin() * f_len), 
                Vec2::new(angle.cos() * (f_len + s_len), angle.sin() * (f_len + s_len))
            ]
        }
    }

    pub fn two_link_solver(&mut self) {
        // (third page of paper)
        // Prevent an error when the start_to_end_distance is greater than the arm_length, making the
        // theta_1 result in NaN.
        // I do believe this happens because though i've changed the slider, the change only takes effect
        // in the next frame. Plus, it can't solve when distance is greater than sum of arm lengths
        if self.first_to_last_dist > self.first_arm_length + self.second_arm_length { 
            // When out of range further than the sum
            self.first_to_last_dist = self.first_arm_length + self.second_arm_length;
        } 
        else if self.first_to_last_dist < self.first_arm_length - self.second_arm_length && self.first_arm_length > self.second_arm_length { 
            // When out of range closer to base (Vec2::ZERO)
            self.first_to_last_dist = self.first_arm_length - self.second_arm_length;
        }
        // Theta_1, i believe, is the angle between the first arm and angle
        // The sum comes from the fact that "angle" is the angle between the horizontal axis and the end effector
        // Law of cosines
        let numerator = self.first_arm_length.powi(2) + self.first_to_last_dist.powi(2) - self.second_arm_length.powi(2);
        let denominator = 2. * self.first_arm_length * self.first_to_last_dist;
        let theta_1 = (numerator / denominator).acos();
        // WARNING: can't reach the base when first_arm_lenth <= second_arm_length

        // First Joint
        let first_p = Vec2::new((theta_1 + self.angle).cos() * self.first_arm_length, (theta_1 + self.angle).sin() * self.first_arm_length);

        // End effector
        let second_p = Vec2::new(self.angle.cos() * self.first_to_last_dist, self.angle.sin() * self.first_to_last_dist);

        self.points = [Vec2::ZERO, first_p, second_p]
    }
}