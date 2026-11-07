use std::{fs, mem, ptr};
use glfw::Context;
use gl::types::*;
use glam::{Vec3, Mat4};
use std::{env, ffi::CString};
use x11::xlib;

mod things;
mod limbs;
mod arthropods;

use arthropods::{CellarSpider, HouseSpider, Harvestman, CraneFly};

fn compile_shader(kind: GLenum, source: &str) -> Result<GLuint, String> {
    let c_src = CString::new(source).map_err(|e| e.to_string())?;

    unsafe {
        let shader = gl::CreateShader(kind);
        gl::ShaderSource(shader, 1, &c_src.as_ptr(), ptr::null());
        gl::CompileShader(shader);

        let mut success: GLint = 0;
        gl::GetShaderiv(shader, gl::COMPILE_STATUS, &mut success);
        if success == 0 {
            let mut len: GLint = 0;
            gl::GetShaderiv(shader, gl::INFO_LOG_LENGTH, &mut len);

            let mut buf = vec![0u8; len.max(1) as usize];
            let mut written: GLsizei = 0;
            gl::GetShaderInfoLog(shader, len, &mut written, buf.as_mut_ptr() as *mut GLchar);
            buf.truncate(written as usize);

            gl::DeleteShader(shader);
            return Err(String::from_utf8_lossy(&buf).into_owned());
        }
        Ok(shader)
    }
}

fn link_program(vertex: GLuint, fragment: GLuint) -> Result<GLuint, String> {
    unsafe {
        let program = gl::CreateProgram();
        gl::AttachShader(program, vertex);
        gl::AttachShader(program, fragment);
        gl::LinkProgram(program);

        let mut success: GLint = 0;
        gl::GetProgramiv(program, gl::LINK_STATUS, &mut success);
        if success == 0 {
            let mut len: GLint = 0;
            gl::GetProgramiv(program, gl::INFO_LOG_LENGTH, &mut len);

            let mut buf = vec![0u8; len.max(1) as usize];
            let mut written: GLsizei = 0;
            gl::GetProgramInfoLog(program, len, &mut written, buf.as_mut_ptr() as *mut GLchar);
            buf.truncate(written as usize);

            gl::DeleteProgram(program);
            return Err(String::from_utf8_lossy(&buf).into_owned());
        }
        Ok(program)
    }
}

fn create_shader_program(vertex_path: &str, fragment_path: &str) -> Result<GLuint, String> {
    let vertex_src = fs::read_to_string(vertex_path)
        .map_err(|e| format!("{vertex_path}: {e}"))?;
    let fragment_src = fs::read_to_string(fragment_path)
        .map_err(|e| format!("{fragment_path}: {e}"))?;

    if vertex_src.is_empty() || fragment_src.is_empty() {
        return Err("empty shader source".to_string());
    }

    let vertex_shader = compile_shader(gl::VERTEX_SHADER, &vertex_src)
        .map_err(|e| format!("vertex compilation error:\n{e}"))?;

    let fragment_shader = match compile_shader(gl::FRAGMENT_SHADER, &fragment_src) {
        Ok(s) => s,
        Err(e) => {
            unsafe { gl:: DeleteShader(vertex_shader) };
            return Err(format!("fragment compilation error:\n{e}"));
        }
    };

    let program = link_program(vertex_shader, fragment_shader)
        .map_err(|e| format!("linking error:\n{e}"));

    unsafe {
        gl::DeleteShader(vertex_shader);
        gl::DeleteShader(fragment_shader);
    }

    program
}

fn main() {
    let argv: Vec<String> = env::args().collect();
    let argc = argv.len();

    let embedded: bool = argc > 1;
    let mut parent: xlib::Window = 0;
    let mut x11dpy: *mut xlib::Display = ptr::null_mut();
    let mut window_width = 800;
    let mut window_height = 600;

    if embedded {
        unsafe {
            parent = parse_int::parse(&argv[1]).expect("invalid window id sucka"); 
            x11dpy = xlib::XOpenDisplay(ptr::null());
            if x11dpy.is_null() {
                return;
            }
            let mut attributes: xlib::XWindowAttributes = mem::zeroed();
            xlib::XGetWindowAttributes(x11dpy, parent, &mut attributes);
            window_width = attributes.width;
            window_height = attributes.height;
        }
    }

    let mut glfw = glfw::init(glfw::fail_on_errors).unwrap();
    glfw.window_hint(glfw::WindowHint::ContextVersion(3, 3));
    glfw.window_hint(glfw::WindowHint::OpenGlProfile(glfw::OpenGlProfileHint::Core));

    if embedded {
        glfw.window_hint(glfw::WindowHint::Visible(false));
        glfw.window_hint(glfw::WindowHint::Decorated(false));
        glfw.window_hint(glfw::WindowHint::Focused(false));
    }

    let (mut window, _events) = glfw.create_window(800, 600, "arthropods", glfw::WindowMode::Windowed).unwrap();

    if embedded {
        unsafe {
            let native: xlib::Window = window.get_x11_window() as xlib::Window;
            xlib::XReparentWindow(x11dpy, native, parent, 0, 0);
            xlib::XResizeWindow(x11dpy, native, window_width as u32, window_height as u32);
            xlib::XMapWindow(x11dpy, native);
            xlib::XFlush(x11dpy);
        }
    }

    window.make_current();
    window.set_framebuffer_size_polling(true);
    gl::load_with(|s| {
        window.get_proc_address(s).map_or(std::ptr::null(), |f| f as *const _)
    });

    let (fb_w, fb_h) = window.get_framebuffer_size();
    unsafe { gl::Viewport(0, 0, fb_w, fb_h) };

    unsafe { gl::Enable(gl::DEPTH_TEST) };

    let mut shader_program;

    if embedded {
        shader_program = match create_shader_program("src/vertex_shader.glsl", "src/fragment_shader.glsl") {
            Ok(p) => p,
            Err(e) => {
                eprintln!("{e}");
                return;
            }
        };
    }
    else {
        shader_program = match create_shader_program("vertex_shader.glsl", "fragment_shader.glsl") {
            Ok(p) => p,
            Err(e) => {
                eprintln!("{e}");
                return;
            }
        };
    }

    let model_name = CString::new("model").unwrap();
    let model_loc: GLint = unsafe { gl::GetUniformLocation(shader_program, model_name.as_ptr()) };
    let view_name = CString::new("view").unwrap();
    let view_loc: GLint = unsafe { gl::GetUniformLocation(shader_program, view_name.as_ptr()) };
    let projection_name = CString::new("projection").unwrap();
    let projection_loc: GLint = unsafe { gl::GetUniformLocation(shader_program, projection_name.as_ptr()) };

    unsafe { gl::PolygonMode(gl::FRONT_AND_BACK, gl::FILL) };

    let mut delta_time;
    let mut last_time = 0.0;

    let mut cellar_spider = CellarSpider::new(-20.0, 0.0, 20.0);

    let mut house_spider = HouseSpider::new(-20.0, 0.0, -20.0);

    let mut harvestman = Harvestman::new(20.0, 0.0, 20.0);

    let mut crane_fly = CraneFly::new(20.0, 0.0, -20.0);

    while !window.should_close() {
        unsafe {
            gl::ClearColor(0.133 ,0.141, 0.212, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
            gl::UseProgram(shader_program);
        }

        let fov = 45.0_f32.to_radians();
        let (fb_w, fb_h) = window.get_framebuffer_size();
        let aspect = fb_w as f32 / fb_h as f32;
        let near = 0.1_f32;
        let far = 1000.0_f32;

        let view = Mat4::from_rotation_x(45.0_f32.to_radians())
            * Mat4::from_translation(Vec3::new(0.0, -75.0, -87.5));

        let projection = glam::camera::rh::proj::opengl::perspective(fov, aspect, near, far);

        unsafe {
            gl::UniformMatrix4fv(view_loc, 1, gl::FALSE, view.to_cols_array().as_ptr());
            gl::UniformMatrix4fv(projection_loc, 1, gl::FALSE, projection.to_cols_array().as_ptr());
        }

        let current_time: f32 = glfw.get_time() as f32;
        delta_time = current_time - last_time;
        last_time = current_time;

        cellar_spider.behave(delta_time);
        cellar_spider.update(delta_time);
        cellar_spider.draw(model_loc); 

        house_spider.behave(delta_time);
        house_spider.update(delta_time);
        house_spider.draw(model_loc);

        harvestman.behave(delta_time);
        harvestman.update(delta_time);
        harvestman.draw(model_loc);

        crane_fly.behave(delta_time);
        crane_fly.update(delta_time);
        crane_fly.draw(model_loc);

        window.swap_buffers();
        glfw.poll_events();
    }

    if embedded {
        unsafe {
            xlib::XCloseDisplay(x11dpy);
        }
    }
}
