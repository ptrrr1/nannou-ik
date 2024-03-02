use nannou::prelude::*;
use nannou_egui::{self, egui::{self, Color32, Rounding, Stroke, Visuals}, Egui};

#[derive(Debug)]
struct Settings {
    arm_length: f32,
    arm_angle: f32,
    start_to_end_distance: f32,
    points: [Vec2; 3],
    target_mouse: bool
}

struct Model {
    _window_id: WindowId,
    clear_color: Hsl,
    egui: Egui,
    settings: Settings,
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
        arm_length: 128.,
        arm_angle: 0.,
        start_to_end_distance: 256.,
        points: [Vec2::ZERO, Vec2::new(128., 0.), Vec2::new(256., 0.)],
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
    draw.polyline()
        .weight(8.0)
        .color(PLUM)
        .points(_model.settings.points);

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
        _ui.checkbox(&mut settings.target_mouse, "Target mouse");

        _ui.separator();

        //
        _ui.label("Arm arm_length - 0 to 128");
        _ui.add(egui::Slider::new(&mut settings.arm_length, 0.001..=128.));

        _ui.separator();

        //
        _ui.label("Start to End distance - 0 to ".to_owned() + &(settings.arm_length * 2.0).to_string());
        _ui.add(egui::Slider::new(&mut settings.start_to_end_distance, 0.001..=2.*settings.arm_length));

        _ui.separator();

        //
        _ui.label("Arm angle - 0 to 6.28 radians");
        _ui.add(egui::Slider::new(&mut settings.arm_angle, 0.0..=2.*PI));
    });

    // IK targeting
    // Automatic
    if settings.target_mouse {
        let mouse_pos = _app.mouse.position();

        settings.start_to_end_distance = Vec2::ZERO.distance(mouse_pos);

        if settings.start_to_end_distance > 2.* settings.arm_length {
            settings.start_to_end_distance = 2. * settings.arm_length;
        }

        // Update with the changes
        _model.settings.points = solve_ik_simple(settings.start_to_end_distance, settings.arm_length, mouse_pos.angle());
    }

    // Manual 
    else {
        // Prevent an error when the start_to_end_distance is greater than the arm_length, making the
        // theta_1 result in NaN.
        // I do believe this happens because though i've changed the slider, the change only takes effect
        // in the next frame.
        if (settings.start_to_end_distance - 2. * settings.arm_length) > 0.0 {
            settings.start_to_end_distance = 2. * settings.arm_length;
        }
        
        // Update with the changes
        _model.settings.points = solve_ik_simple(settings.start_to_end_distance, settings.arm_length, settings.arm_angle);
    }
}

fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Let egui handle things like keyboard and mouse input.
    model.egui.handle_raw_event(event);
}

fn solve_ik_simple(distance: f32, length: f32, angle: f32) -> [Vec2; 3] {
    // Theta_1, i believe, is the angle between the first arm and the horizontal axis
    // ain't sure since i need to sum theta with the arm_angle
    // Also, this is simplified because it assumes both arms have equal arm_length
    let theta_1 = (distance / (2. * length)).acos();

    // First Joint
    let first_p = Vec2::new((theta_1 + angle).cos() * length, (theta_1 + angle).sin() * length);

    // End effector || Second Joint
    let second_p = Vec2::new(angle.cos() * distance, angle.sin() * distance);

    return [Vec2::ZERO, first_p, second_p];
}