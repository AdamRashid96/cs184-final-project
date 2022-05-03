#ifndef CGL_CLOTH_SIMULATOR_H
#define CGL_CLOTH_SIMULATOR_H

#include <nanogui/nanogui.h>
#include <memory>
#include "misc/sphere_drawing.h"
#include "camera.h"
#include "collision/collisionObject.h"
#include "particle.h"
#include "gasField.h"

using namespace nanogui;

struct UserShader;
enum ShaderTypeHint { WIREFRAME = 0, NORMALS = 1, PHONG = 2 };

class Simulator {
public:
  Simulator(std::string project_root, Screen *screen);
  ~Simulator();

  void init();

  void loadCollisionObjects(vector<CollisionObject *> *objects);
  virtual bool isAlive();
  virtual void drawContents();

  // Screen events

  virtual bool cursorPosCallbackEvent(double x, double y);
  virtual bool mouseButtonCallbackEvent(int button, int action, int modifiers);
  virtual bool keyCallbackEvent(int key, int scancode, int action, int mods);
  virtual bool dropCallbackEvent(int count, const char **filenames);
  virtual bool scrollCallbackEvent(double x, double y);
  virtual bool resizeCallbackEvent(int width, int height);

private:
  virtual void initGUI(Screen *screen);
  void drawLines(GLShader &shader);
  void drawWireframe(GLShader &shader);
  void drawNormals(GLShader &shader);
  void drawPhong(GLShader &shader);
  void explosion(double frames_per_sec, double simulation_steps,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects);
  
  void load_shaders();
  void load_textures();
  
  void initParticles();
  void particleSimulate(double frames_per_sec, double simulation_steps,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects);

  void time_step(double delta_time);
  void particle_time_step(double delta_time);
  void fluid_time_step(double delta_time);

  void field_vel_step(Vector3D* v0, float visc, double delta_time);

  // File management
  
  std::string m_project_root;

  // Camera methods

  virtual void resetCamera();
  virtual Matrix4f getProjectionMatrix();
  virtual Matrix4f getViewMatrix();

  // Default simulation values

  int frames_per_sec = 90;
  int simulation_steps = 30;

  CGL::Vector3D gravity = CGL::Vector3D(0, -9.8, 0);
  nanogui::Color color = nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f);

  vector<CollisionObject *> *collision_objects;
  vector<Particle *> *particles;

  // Gas Field Parameters
  int field_width = 30;
  int field_height = 30;
  int field_depth = 30;
  double field_cell_size = 0.1;
  double field_viscosity = 0;

  double ambient_temperature = 0; // Kelvin
  double base_pressure = 0;
  double initial_velocity = 0.01;

  GasField field;

  vector<Vector3D> line_endpoints;
  int wireframe_shader_idx = 0;

  // TODO: Tune these values
  double a_h = 0.9;
  double a_d = 0.9;
  double particle_mass_threshold = 0;
  double particle_thermal_mass_threshold = 0;
  double ignition_temperature;
  double burn_rate;
  double b_h;
  double b_s;
  double b_g;
  double mass_creation_threshold;

  // Explosion parameters
  int num_particles = 10000;
  double explosion_radius = .1;
  double particle_radius = 0.01;
  double particle_density = 1;
  double max_vel = 10;
  double min_vel = 5;

  // OpenGL attributes

  int active_shader_idx = 6;

  vector<UserShader> shaders;
  vector<std::string> shaders_combobox_names;
  
  // OpenGL textures
  
  Vector3D m_gl_texture_1_size;
  Vector3D m_gl_texture_2_size;
  Vector3D m_gl_texture_3_size;
  Vector3D m_gl_texture_4_size;
  GLuint m_gl_texture_1;
  GLuint m_gl_texture_2;
  GLuint m_gl_texture_3;
  GLuint m_gl_texture_4;
  GLuint m_gl_cubemap_tex;
  
  // OpenGL customizable inputs
  
  double m_normal_scaling = 2.0;
  double m_height_scaling = 0.1;

  // Camera attributes

  CGL::Camera camera;
  CGL::Camera canonicalCamera;

  double view_distance;
  double canonical_view_distance;
  double min_view_distance;
  double max_view_distance;

  double scroll_rate;

  // Screen methods

  Screen *screen;
  void mouseLeftDragged(double x, double y);
  void mouseRightDragged(double x, double y);
  void mouseMoved(double x, double y);

  // Mouse flags

  bool left_down = false;
  bool right_down = false;
  bool middle_down = false;

  // Keyboard flags

  bool ctrl_down = false;

  // Simulation flags

  bool is_paused = true;

  // Screen attributes

  int mouse_x;
  int mouse_y;

  int screen_w;
  int screen_h;

  bool is_alive = true;

  Vector2i default_window_size = Vector2i(1024, 800);

  // Sphere mesh
  int num_lat = 5;
  int num_lon = 5;
  Misc::SphereMesh sphere_mesh = Misc::SphereMesh(num_lat, num_lon);
};

struct UserShader {
  UserShader(std::string display_name, std::shared_ptr<GLShader> nanogui_shader, ShaderTypeHint type_hint)
  : display_name(display_name)
  , nanogui_shader(nanogui_shader)
  , type_hint(type_hint) {
  }
  
  std::shared_ptr<GLShader> nanogui_shader;
  std::string display_name;
  ShaderTypeHint type_hint;
  
};

#endif // CGL_CLOTH_SIM_H
