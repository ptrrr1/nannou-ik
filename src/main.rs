// The paper: Inverse Kinematics Techniques in Computer Graphics: A Survey by A. Aristidou, J.Lasenby, Y. Chrysanthou1 and A. Shamir
// Plus a video titled Learn Inverse Kinematics the Simple Way by Benjamin Blodgett --> https://youtu.be/wgpgNLEEpeY
// This video helped a ton

use nannou::prelude::*;
use nannou_egui::{self, egui::{self, Color32, Rounding, Stroke, Visuals}, Egui};

#[derive(Debug, Clone, Copy)]
enum Solver {
    Analytic(AnalyticTwoLink),
}

#[derive(Debug, Clone, Copy)]
struct AnalyticTwoLink {
    angle: f32,
    first_arm_length: f32,
    second_arm_length: f32,
    first_to_last_dist: f32,
    points: [Vec2; 3],
}

impl AnalyticTwoLink {
    fn new(angle: f32, f_len: f32, s_len: f32) -> AnalyticTwoLink {
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

    fn two_link_solver(&mut self) {
        // Prevent an error when the start_to_end_distance is greater than the arm_length, making the
        // theta_1 result in NaN.
        // I do believe this happens because though i've changed the slider, the change only takes effect
        // in the next frame. Plus, it can't solve when distance is greater than sum of arm lengths
        if self.first_to_last_dist > self.first_arm_length + self.second_arm_length {
            self.first_to_last_dist = self.first_arm_length + self.second_arm_length;
        } 
        else if self.first_to_last_dist < self.first_arm_length - self.second_arm_length {
            self.first_to_last_dist = self.first_arm_length - self.second_arm_length;
        }
        // Theta_1, i believe, is the angle between the first arm and angle
        // The sum comes from the fact that "angle" is the angle between the horizontal axis and the end effector
        // Law of cosines
        let mut numerator = self.first_arm_length.powi(2) + self.first_to_last_dist.powi(2) - self.second_arm_length.powi(2);
        let denominator = 2. * self.first_arm_length * self.first_to_last_dist;
        while numerator.abs() > denominator {
             numerator = denominator - numerator;
        }
        let theta_1 = (numerator / denominator).acos();
        println!("{:?} / {:?}", numerator, denominator);

        // First Joint
        let first_p = Vec2::new((theta_1 + self.angle).cos() * self.first_arm_length, (theta_1 + self.angle).sin() * self.first_arm_length);

        // End effector
        let second_p = Vec2::new(self.angle.cos() * self.first_to_last_dist, self.angle.sin() * self.first_to_last_dist);

        self.points = [Vec2::ZERO, first_p, second_p]
    }
}

struct Model {
    _window_id: WindowId,
    clear_color: Hsl,
    egui: Egui,
    settings: Settings,
}

#[derive(Debug)]
struct Settings {
    solver: Solver,
    target_mouse: bool,
}

fn main() {
    nannou::app(model)
        .view(view) // The function that will be called for presenting graphics to a frame.
        .update(update) // Called every frame
        .run();
}

fn model(_app: &App) -> Model {
    // Create a new window! Store the ID so we can refer to it later.
    let _window_id = _app
        .new_window()
        .title("nannou-app | IK simulation")
        .size(512u32, 512u32)
        .resizable(false)
        .raw_event(raw_window_event)
        .build().unwrap();

    let clear_color = hsl(0., 0., 0.);

    let window = _app.window(_window_id).unwrap();
    let egui = Egui::from_window(&window);

    let settings = Settings {
        solver: Solver::Analytic(AnalyticTwoLink::new(0., 128., 128.)),
        target_mouse: false,
    };

    Model { 
        _window_id,
        clear_color,
        egui,
        settings,
    }
}

fn view(_app: &App, _model: &Model, _frame: Frame) {
    let draw = _app.draw();
    
    // Background color
    draw.background().color(_model.clear_color);

    //
    match &_model.settings.solver {
        Solver::Analytic(two_link_chain) => {
            draw.polyline()
            .weight(8.0)
            .color(PLUM)
            .points(two_link_chain.points);
            }
    };

    // Draw to frame
    draw.to_frame(_app, &_frame).unwrap();

    // Draw GUI
    _model.egui.draw_to_frame(&_frame).unwrap();
}

fn update(_app: &App, _model: &mut Model, _update: Update) {
    // Values i wish to change or show through UI
    let settings = &mut _model.settings;

    // GUI thingies
    let _egui = &mut _model.egui;
    _egui.set_elapsed_time(_update.since_start);

    let _ctx = _egui.begin_frame();
    _ctx.set_visuals( Visuals { 
        dark_mode: true, 
        window_rounding: Rounding { nw: 0., ne: 0., sw: 0., se: 0. }, 
        window_fill: Color32::from_rgb(0, 0, 0),
        window_stroke: Stroke::new(1., Color32::WHITE),
        slider_trailing_fill: true,
        ..Default::default() 
    });
    
    // Creates new UI of type Window
    egui::Window::new("IK Arm Settings")
        .default_size((170., 200.))
        .resizable(false)
        .show(&_ctx, |_ui| {
         //
         _ui.add(egui::Checkbox::new(&mut settings.target_mouse, "Target mouse"));
         _ui.add(egui::Separator::default());

        //
        match settings.solver {
            // Feels kinda weird that i have to "ref mut" it so the changes inside are persistent outside
            Solver::Analytic(ref mut two_link_chain) => {
                //
                _ui.add(egui::Label::new("First Arm Length"));
                _ui.add(egui::Slider::new(&mut two_link_chain.first_arm_length, 0.001..=256.).max_decimals(2));
                _ui.add(egui::Separator::default());

                //
                _ui.add(egui::Label::new("Second Arm Length"));
                _ui.add(egui::Slider::new(&mut two_link_chain.second_arm_length, 0.001..=256.).max_decimals(2));
                _ui.add(egui::Separator::default());

                //
                _ui.add(egui::Label::new("Base to End effector distance"));
                _ui.add(egui::Slider::new(&mut two_link_chain.first_to_last_dist, 0.001..=(two_link_chain.first_arm_length + two_link_chain.second_arm_length)).max_decimals(2));
                _ui.add(egui::Separator::default());

                //
                _ui.add(egui::Label::new("Arm angle"));
                _ui.add(egui::Slider::new(&mut two_link_chain.angle, 0.0..=2.*PI).min_decimals(2));
            }
        };
    });
    // IK targeting
    // Automatic
    if settings.target_mouse {
        let mouse_pos = _app.mouse.position();

        match settings.solver {
            Solver::Analytic(ref mut two_link_chain) => {
                two_link_chain.first_to_last_dist = Vec2::ZERO.distance(mouse_pos);
                two_link_chain.angle = mouse_pos.angle();

                two_link_chain.two_link_solver()
            }
        }
    }
    // Manual 
    else 
    {    
        match settings.solver {
            Solver::Analytic(ref mut two_link_chain) => { two_link_chain.two_link_solver() }
        }
    }

    // Update with the changes
    _model.settings.solver = settings.solver;
}

fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Let egui handle things like keyboard and mouse input.
    model.egui.handle_raw_event(event);
}