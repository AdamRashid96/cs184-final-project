#include <cmath>
#include <glad/glad.h>

#include <CGL/vector3D.h>
#include "util/random_util.h"
#include <nanogui/nanogui.h>

#include "simulator.h"

#include "camera.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "misc/camera_info.h"
#include "misc/file_utils.h"
// Needed to generate stb_image binaries. Should only define in exactly one source file importing stb_image.h.
#define STB_IMAGE_IMPLEMENTATION
#include "misc/stb_image.h"

using namespace nanogui;
using namespace std;

Vector3D load_texture(int frame_idx, GLuint handle, const char* where) {
  Vector3D size_retval;
  
  if (strlen(where) == 0) return size_retval;
  
  glActiveTexture(GL_TEXTURE0 + frame_idx);
  glBindTexture(GL_TEXTURE_2D, handle);
  
  
  int img_x, img_y, img_n;
  unsigned char* img_data = stbi_load(where, &img_x, &img_y, &img_n, 3);
  size_retval.x = img_x;
  size_retval.y = img_y;
  size_retval.z = img_n;
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_x, img_y, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  stbi_image_free(img_data);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  return size_retval;
}

void load_cubemap(int frame_idx, GLuint handle, const std::vector<std::string>& file_locs) {
  glActiveTexture(GL_TEXTURE0 + frame_idx);
  glBindTexture(GL_TEXTURE_CUBE_MAP, handle);
  for (int side_idx = 0; side_idx < 6; ++side_idx) {
    
    int img_x, img_y, img_n;
    unsigned char* img_data = stbi_load(file_locs[side_idx].c_str(), &img_x, &img_y, &img_n, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + side_idx, 0, GL_RGB, img_x, img_y, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
    stbi_image_free(img_data);
    std::cout << "Side " << side_idx << " has dimensions " << img_x << ", " << img_y << std::endl;

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  }
}

void Simulator::load_textures() {
  glGenTextures(1, &m_gl_texture_1);
  glGenTextures(1, &m_gl_texture_2);
  glGenTextures(1, &m_gl_texture_3);
  glGenTextures(1, &m_gl_texture_4);
  glGenTextures(1, &m_gl_cubemap_tex);
  
  m_gl_texture_1_size = load_texture(1, m_gl_texture_1, (m_project_root + "/textures/texture_1.png").c_str());
  m_gl_texture_2_size = load_texture(2, m_gl_texture_2, (m_project_root + "/textures/texture_2.png").c_str());
  m_gl_texture_3_size = load_texture(3, m_gl_texture_3, (m_project_root + "/textures/texture_3.png").c_str());
  m_gl_texture_4_size = load_texture(4, m_gl_texture_4, (m_project_root + "/textures/texture_4.png").c_str());
  
  std::cout << "Texture 1 loaded with size: " << m_gl_texture_1_size << std::endl;
  std::cout << "Texture 2 loaded with size: " << m_gl_texture_2_size << std::endl;
  std::cout << "Texture 3 loaded with size: " << m_gl_texture_3_size << std::endl;
  std::cout << "Texture 4 loaded with size: " << m_gl_texture_4_size << std::endl;
  
  std::vector<std::string> cubemap_fnames = {
    m_project_root + "/textures/cube/posx.jpg",
    m_project_root + "/textures/cube/negx.jpg",
    m_project_root + "/textures/cube/posy.jpg",
    m_project_root + "/textures/cube/negy.jpg",
    m_project_root + "/textures/cube/posz.jpg",
    m_project_root + "/textures/cube/negz.jpg"
  };
  
  load_cubemap(5, m_gl_cubemap_tex, cubemap_fnames);
  std::cout << "Loaded cubemap texture" << std::endl;
}

void Simulator::initParticles() {
  particles = new vector<Particle *>(num_particles);

  for (int i = 0; i < num_particles; i++) {
    Vector3D pos = get_sample() * explosion_radius * random_uniform();
    (*particles)[i] = new Particle(FUEL, fuel_density, fuel_specific_heat_capacity, pos, particle_radius, particle_base_temperature);

    //(*particles)[i]->velocity = pos.unit() * (min_vel + random_uniform() * (max_vel - min_vel));
  }
}

void Simulator::particleSimulate(double frames_per_sec, double simulation_steps,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // // TODO (Part 4): Handle self-collisions.
  // build_spatial_map();
  // for (int i = 0; i < point_masses.size(); i++) {
  //   self_collide(point_masses[i], simulation_steps);
  // }


  // // TODO (Part 3): Handle collisions with other primitives.
  // for (int k = 0; k < collision_objects->size(); k++) {
  //   for (int i = 0; i < num_height_points; i++) {
  //     for (int j = 0; j < num_width_points; j++) {
  //       int idx = (i * num_width_points) + j;
  //       (*collision_objects)[k]->collide(point_masses[idx]);
  //     }
  //   }
  // }
}

void Simulator::time_step(double delta_time) {
  // Particle Gravity
  for (int i = 0; i < particles->size(); i++) {
    Particle* particle = (*particles)[i];
    particle->force = particle->mass() * gravity;
  }

  // Fluid Bouyancy and Vorticity Confinement
  for (int i = 1; i <= field.width; i++) {
    for (int j = 1; j <= field.height; j++) {
      for (int k = 1; k <= field.depth; k++) {
        FieldCell* cell = field.CellAt(field.cells, i, j, k);

        // Thermal buoyancy force
        cell->force = (-alpha * cell->pressure + beta * (cell->temperature - ambient_temperature)) * Vector3D(0, 1, 0);
      }
    }
  }

  // Vorticity Confinement
  field.vorticity_confinement(epsilon);

  // Particle Fluid Interaction
  for (int i = 0; i < particles->size(); i++) {
    Particle* particle = (*particles)[i];
    // Sample the fluid at the particle position to get interpolated cell values
    FieldCell interpolatedCell = field.CellAtInterpolated(particle->position);
    Vector3D velocityDifference = interpolatedCell.velocity - particle->velocity;
    double r2 = particle->radius * particle->radius;

    // Calculate force and heat transfer
    Vector3D force;
    if (particle->mass() >= particle_mass_threshold) {
      if (particle->type == FUEL) {
        force = a_d * r2 * velocityDifference * velocityDifference.norm();
      } else {

        force = 13 * r2 * velocityDifference * velocityDifference.norm();
      }
    } else {
      force = Vector3D(0);
    }

    double heat_transfer;
    if (particle->thermal_mass() >= particle_thermal_mass_threshold) {
      heat_transfer = a_h * r2 * (interpolatedCell.temperature - particle->temperature);
    } else {
      heat_transfer = 0;
    }

    FieldCell* cell = field.CellAt(particle->position);
    if (cell != NULL) {
      cell->force -= force;
      cell->heat_transfer -= heat_transfer;
    }
    particle->force += force;
    particle->heat_transfer += heat_transfer;
  }

  vector<int> deleted_particles;
  vector<Particle *> created_particles;


  // Fuel Particle Burning
  for (int i = 0; i < particles->size(); i++) {
    Particle* particle = (*particles)[i];
    if (particle->type == FUEL && particle->temperature >= ignition_temperature) {
      double burned_mass = burn_rate * delta_time;

      bool deleted = false;

      // Consume burned mass
      double new_mass = particle->mass() - burned_mass;
      if (new_mass <= 0.0) {
        // Delete the particle
        deleted_particles.push_back(i);
        deleted = true;
      } else {
        particle->setMass(new_mass);
      }

      // Heat generation
      particle->heat_transfer += b_h * burned_mass;

      // Mass generation
      particle->soot_mass += b_s * burned_mass;
      if (particle->soot_mass >= mass_creation_threshold || deleted) {
        // Create soot particle
        Vector3D position = particle->position + particle->radius * (get_sample() * 2 + random_uniform());
        
        Particle* soot = new Particle(SOOT, soot_density, soot_specific_heat_capacity, position, 0.001, particle->temperature);
        soot->setMass(particle->soot_mass);
        soot->velocity = particle->velocity;
        particle->soot_mass = 0.0;

        created_particles.push_back(soot);
      }

      FieldCell* cell = field.CellAt(particle->position);
      cell->phi += b_g * burned_mass / (field.cell_size * field.cell_size * field.cell_size);
    }
  }

  // Delete the particles
  for (int i = deleted_particles.size() - 1; i >= 0; i--) {
    particles->erase(particles->begin() + deleted_particles[i]);
  }

  // Create the particles
  for (int i = 0; i < created_particles.size(); i++) {
    Particle* particle = created_particles[i];
    particles->push_back(particle);
  }

  // Explosion Time Step
  explosion_time_step(delta_time);

  // Fluid Time Step
  fluid_time_step(delta_time);

  // Particle Time Step
  particle_time_step(delta_time);
}

void Simulator::explosion_time_step(double delta_time) {
  elapsed_time += delta_time;

  // if (elapsed_time < 10) {
  //   for (int i = 1; i <= field.width; i++) {
  //     for (int j = 1; j <= field.height; j++) {
  //       for (int k = 1; k <= field.depth; k++) {
  //         double distance = field.CellPos(i, j, k).norm();
  //         if (distance < explosion_radius) {
  //           field.CellAt(i, j, k)->force = Vector3D(0.2, 100, 0.4);
  //         }
  //       }
  //     }
  //   }
  // }
    

  // return;


  double phi = 2000 * pow(2, - 8 * elapsed_time) * sin(10 * elapsed_time);

  for (int i = 1; i <= field.width; i++) {
    for (int j = 1; j <= field.height; j++) {
      for (int k = 1; k <= field.depth; k++) {
        double distance = field.CellPos(i, j, k).norm();
        if (distance < explosion_radius) {

          if (elapsed_time < 0) {
            field.CellAt(i, k, k)->phi = 300;
          } else {
            field.CellAt(i, k, k)->phi = 300 * pow(2, - (elapsed_time - 0.0) / 1.0);
            field.CellAt(i, k, k)->heat_transfer += 1000000;
          }
          
        }
      }
    }
  }
}

void Simulator::particle_time_step(double delta_time) {
  for (int i = 0; i < particles->size(); i++) {
    Particle* particle = (*particles)[i];

    double mass = particle->mass();
    if (mass >= particle_mass_threshold) {
      Vector3D velocity = particle->velocity;
      Vector3D position = particle->position;
      particle->position += velocity * delta_time + particle->force / mass * delta_time * delta_time;
      particle->velocity += particle->force / mass * delta_time;
    }

    if (particle->thermal_mass() >= particle_thermal_mass_threshold) {
      particle->temperature += particle->heat_transfer / particle->thermal_mass() * delta_time;
      //std::cout << particle->temperature << std::endl;
    }
  }
}

void Simulator::fluid_time_step(double delta_time) {
  field.apply_heat(delta_time, ambient_temperature, max_temperature, c_v, c_r);
  field.temperature_diffusion(c_k, delta_time);
  field.temperature_advection(delta_time);

  //field.project();
  field.apply_force(delta_time);
  field.advect(delta_time);
  //field.project();
  field.pressure_step(delta_time);
}

void Simulator::load_shaders() {
  std::set<std::string> shader_folder_contents;
  bool success = FileUtils::list_files_in_directory(m_project_root + "/shaders", shader_folder_contents);
  if (!success) {
    std::cout << "Error: Could not find the shaders folder!" << std::endl;
  }
  
  std::string std_vert_shader = m_project_root + "/shaders/Default.vert";
  
  for (const std::string& shader_fname : shader_folder_contents) {
    std::string file_extension;
    std::string shader_name;
    
    FileUtils::split_filename(shader_fname, shader_name, file_extension);
    
    if (file_extension != "frag") {
      std::cout << "Skipping non-shader file: " << shader_fname << std::endl;
      continue;
    }
    
    std::cout << "Found shader file: " << shader_fname << std::endl;
    
    // Check if there is a proper .vert shader or not for it
    std::string vert_shader = std_vert_shader;
    std::string associated_vert_shader_path = m_project_root + "/shaders/" + shader_name + ".vert";
    if (FileUtils::file_exists(associated_vert_shader_path)) {
      vert_shader = associated_vert_shader_path;
    }
    
    std::shared_ptr<GLShader> nanogui_shader = make_shared<GLShader>();
    nanogui_shader->initFromFiles(shader_name, vert_shader,
                                  m_project_root + "/shaders/" + shader_fname);
    
    // Special filenames are treated a bit differently
    ShaderTypeHint hint;
    if (shader_name == "Wireframe") {
      hint = ShaderTypeHint::WIREFRAME;
      std::cout << "Type: Wireframe" << std::endl;
    } else if (shader_name == "Normal") {
      hint = ShaderTypeHint::NORMALS;
      std::cout << "Type: Normal" << std::endl;
    } else {
      hint = ShaderTypeHint::PHONG;
      std::cout << "Type: Custom" << std::endl;
    }
    
    UserShader user_shader(shader_name, nanogui_shader, hint);
    
    shaders.push_back(user_shader);
    shaders_combobox_names.push_back(shader_name);
  }
  
  // Assuming that it's there, use "Wireframe" by default
  for (size_t i = 0; i < shaders_combobox_names.size(); ++i) {
    if (shaders_combobox_names[i] == "Billboard") {
      active_shader_idx = i;
    }if (shaders_combobox_names[i] == "Wireframe") {
      wireframe_shader_idx = i;
      break;
    }
  }
}

Simulator::Simulator(std::string project_root, Screen *screen)
: m_project_root(project_root), field(field_width, field_height, field_depth, field_cell_size,
                   field_density, field_base_temperature, 0, initial_velocity) {
  this->screen = screen;
  
  this->load_shaders();
  this->load_textures();

  // Initialize particles
  this->initParticles();

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
}

Simulator::~Simulator() {
  for (auto shader : shaders) {
    shader.nanogui_shader->free();
  }
  glDeleteTextures(1, &m_gl_texture_1);
  glDeleteTextures(1, &m_gl_texture_2);
  glDeleteTextures(1, &m_gl_texture_3);
  glDeleteTextures(1, &m_gl_texture_4);
  glDeleteTextures(1, &m_gl_cubemap_tex);

  if (collision_objects) delete collision_objects;
}

void Simulator::explosion(double frames_per_sec, double simulation_steps,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects){
  double mass = 1;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // Compute total force acting on each point mass.

  for (int i = 0; i < 1; i++) {
    Vector3D origin(i, i, 0);
    Sphere *s = new Sphere(origin, 0.5, 0.5, 40, 40);
    collision_objects->push_back(s);
  }

  // Vector3D external_force;
  // for (int i = 0; i < external_accelerations.size(); i++) {
  //   external_force += mass * external_accelerations[i];
  // }
                    

}

void Simulator::loadCollisionObjects(vector<CollisionObject *> *objects) { this->collision_objects = objects; }

/**
 * Initializes the cloth simulation and spawns a new thread to separate
 * rendering from simulation.
 */
void Simulator::init() {

  // Initialize GUI
  screen->setSize(default_window_size);
  initGUI(screen);

  // Initialize camera

  CGL::Collada::CameraInfo camera_info;
  camera_info.hFov = 50;
  camera_info.vFov = 35;
  camera_info.nClip = 0.01;
  camera_info.fClip = 10000;

  // Try to intelligently figure out the camera target

  Vector3D avg_pm_position(0, 0, 0);

  CGL::Vector3D target(avg_pm_position.x, avg_pm_position.y / 2,
                       avg_pm_position.z);
  CGL::Vector3D c_dir(0., 0., 0.);
  canonical_view_distance = max(1, 1) * 0.9;
  scroll_rate = canonical_view_distance / 10;

  view_distance = canonical_view_distance * 2;
  min_view_distance = canonical_view_distance / 10.0;
  max_view_distance = canonical_view_distance * 20.0;

  // canonicalCamera is a copy used for view resets

  camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
               min_view_distance, max_view_distance);
  canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                        view_distance, min_view_distance, max_view_distance);

  screen_w = default_window_size(0);
  screen_h = default_window_size(1);

  camera.configure(camera_info, screen_w, screen_h);
  canonicalCamera.configure(camera_info, screen_w, screen_h);
}

bool Simulator::isAlive() { return is_alive; }

struct colourSystem {
    char *name;                     /* Colour system name */
    double xRed, yRed,              /* Red x, y */
           xGreen, yGreen,          /* Green x, y */
           xBlue, yBlue,            /* Blue x, y */
           xWhite, yWhite,          /* White point x, y */
           gamma;                   /* Gamma correction for system */
};

void spectrum_to_xyz(double (*spec_intens)(double wavelength),
                     double *x, double *y, double *z)
{
    int i;
    double lambda, X = 0, Y = 0, Z = 0, XYZ;

    /* CIE colour matching functions xBar, yBar, and zBar for
       wavelengths from 380 through 780 nanometers, every 5
       nanometers.  For a wavelength lambda in this range:

            cie_colour_match[(lambda - 380) / 5][0] = xBar
            cie_colour_match[(lambda - 380) / 5][1] = yBar
            cie_colour_match[(lambda - 380) / 5][2] = zBar

        To save memory, this table can be declared as floats
        rather than doubles; (IEEE) float has enough
        significant bits to represent the values. It's declared
        as a double here to avoid warnings about "conversion
        between floating-point types" from certain persnickety
        compilers. */

    static double cie_colour_match[81][3] = {
        {0.0014,0.0000,0.0065}, {0.0022,0.0001,0.0105}, {0.0042,0.0001,0.0201},
        {0.0076,0.0002,0.0362}, {0.0143,0.0004,0.0679}, {0.0232,0.0006,0.1102},
        {0.0435,0.0012,0.2074}, {0.0776,0.0022,0.3713}, {0.1344,0.0040,0.6456},
        {0.2148,0.0073,1.0391}, {0.2839,0.0116,1.3856}, {0.3285,0.0168,1.6230},
        {0.3483,0.0230,1.7471}, {0.3481,0.0298,1.7826}, {0.3362,0.0380,1.7721},
        {0.3187,0.0480,1.7441}, {0.2908,0.0600,1.6692}, {0.2511,0.0739,1.5281},
        {0.1954,0.0910,1.2876}, {0.1421,0.1126,1.0419}, {0.0956,0.1390,0.8130},
        {0.0580,0.1693,0.6162}, {0.0320,0.2080,0.4652}, {0.0147,0.2586,0.3533},
        {0.0049,0.3230,0.2720}, {0.0024,0.4073,0.2123}, {0.0093,0.5030,0.1582},
        {0.0291,0.6082,0.1117}, {0.0633,0.7100,0.0782}, {0.1096,0.7932,0.0573},
        {0.1655,0.8620,0.0422}, {0.2257,0.9149,0.0298}, {0.2904,0.9540,0.0203},
        {0.3597,0.9803,0.0134}, {0.4334,0.9950,0.0087}, {0.5121,1.0000,0.0057},
        {0.5945,0.9950,0.0039}, {0.6784,0.9786,0.0027}, {0.7621,0.9520,0.0021},
        {0.8425,0.9154,0.0018}, {0.9163,0.8700,0.0017}, {0.9786,0.8163,0.0014},
        {1.0263,0.7570,0.0011}, {1.0567,0.6949,0.0010}, {1.0622,0.6310,0.0008},
        {1.0456,0.5668,0.0006}, {1.0026,0.5030,0.0003}, {0.9384,0.4412,0.0002},
        {0.8544,0.3810,0.0002}, {0.7514,0.3210,0.0001}, {0.6424,0.2650,0.0000},
        {0.5419,0.2170,0.0000}, {0.4479,0.1750,0.0000}, {0.3608,0.1382,0.0000},
        {0.2835,0.1070,0.0000}, {0.2187,0.0816,0.0000}, {0.1649,0.0610,0.0000},
        {0.1212,0.0446,0.0000}, {0.0874,0.0320,0.0000}, {0.0636,0.0232,0.0000},
        {0.0468,0.0170,0.0000}, {0.0329,0.0119,0.0000}, {0.0227,0.0082,0.0000},
        {0.0158,0.0057,0.0000}, {0.0114,0.0041,0.0000}, {0.0081,0.0029,0.0000},
        {0.0058,0.0021,0.0000}, {0.0041,0.0015,0.0000}, {0.0029,0.0010,0.0000},
        {0.0020,0.0007,0.0000}, {0.0014,0.0005,0.0000}, {0.0010,0.0004,0.0000},
        {0.0007,0.0002,0.0000}, {0.0005,0.0002,0.0000}, {0.0003,0.0001,0.0000},
        {0.0002,0.0001,0.0000}, {0.0002,0.0001,0.0000}, {0.0001,0.0000,0.0000},
        {0.0001,0.0000,0.0000}, {0.0001,0.0000,0.0000}, {0.0000,0.0000,0.0000}
    };

    for (i = 0, lambda = 380; lambda < 780.1; i++, lambda += 5) {
        double Me;

        Me = (*spec_intens)(lambda);
        X += Me * cie_colour_match[i][0];
        Y += Me * cie_colour_match[i][1];
        Z += Me * cie_colour_match[i][2];
    }
    XYZ = (X + Y + Z);
    *x = X / XYZ;
    *y = Y / XYZ;
    *z = Z / XYZ;
}

void xyz_to_rgb(struct colourSystem *cs,
                double xc, double yc, double zc,
                double *r, double *g, double *b)
{
    double xr, yr, zr, xg, yg, zg, xb, yb, zb;
    double xw, yw, zw;
    double rx, ry, rz, gx, gy, gz, bx, by, bz;
    double rw, gw, bw;

    xr = cs->xRed;    yr = cs->yRed;    zr = 1 - (xr + yr);
    xg = cs->xGreen;  yg = cs->yGreen;  zg = 1 - (xg + yg);
    xb = cs->xBlue;   yb = cs->yBlue;   zb = 1 - (xb + yb);

    xw = cs->xWhite;  yw = cs->yWhite;  zw = 1 - (xw + yw);

    /* xyz -> rgb matrix, before scaling to white. */

    rx = (yg * zb) - (yb * zg);  ry = (xb * zg) - (xg * zb);  rz = (xg * yb) - (xb * yg);
    gx = (yb * zr) - (yr * zb);  gy = (xr * zb) - (xb * zr);  gz = (xb * yr) - (xr * yb);
    bx = (yr * zg) - (yg * zr);  by = (xg * zr) - (xr * zg);  bz = (xr * yg) - (xg * yr);

    /* White scaling factors.
       Dividing by yw scales the white luminance to unity, as conventional. */

    rw = ((rx * xw) + (ry * yw) + (rz * zw)) / yw;
    gw = ((gx * xw) + (gy * yw) + (gz * zw)) / yw;
    bw = ((bx * xw) + (by * yw) + (bz * zw)) / yw;

    /* xyz -> rgb matrix, correctly scaled to white. */

    rx = rx / rw;  ry = ry / rw;  rz = rz / rw;
    gx = gx / gw;  gy = gy / gw;  gz = gz / gw;
    bx = bx / bw;  by = by / bw;  bz = bz / bw;

    /* rgb of the desired point */

    *r = (rx * xc) + (ry * yc) + (rz * zc);
    *g = (gx * xc) + (gy * yc) + (gz * zc);
    *b = (bx * xc) + (by * yc) + (bz * zc);
}

double bbTemp = 5000;                 /* Hidden temperature argument
                                         to BB_SPECTRUM. */
double bb_spectrum(double wavelength)
{
    double wlm = wavelength * 1e-9;   /* Wavelength in meters */

    return (3.74183e-16 * pow(wlm, -5.0)) /
           (exp(1.4388e-2 / (wlm * bbTemp)) - 1.0);
}


void Simulator::drawContents() {
  glEnable(GL_DEPTH_TEST);

  if (!is_paused) {
    double delta_time = 1.0 / frames_per_sec;
    time_step(delta_time);
  }

  // Bind the active shader
  const UserShader& active_shader = shaders[active_shader_idx];
  GLShader &shader = *active_shader.nanogui_shader;
  shader.bind();

  // Prepare the camera projection matrix

  Matrix4f view = getViewMatrix();
  Matrix4f projection = getProjectionMatrix();

  // shader.setUniform("u_model_view", view * model);
  shader.setUniform("u_projection", projection);
  shader.setUniform("u_view", view);

  MatrixXf positions = MatrixXf(4, 6 * particles->size());
  MatrixXf worldPos = MatrixXf(4, 6 * particles->size());
  MatrixXf radii = MatrixXf(1, 6 * particles->size());
  MatrixXf uvs = MatrixXf(2, 6 * particles->size());
  MatrixXf color = MatrixXf(4, 6 * particles->size());

  for (int i = 0; i < particles->size(); i++) {
    Particle* particle = (*particles)[i];

      // Matrix4f model;
      // model << particle->radius, 0, 0, particle->position.x, 0, particle->radius, 0, particle->position.y, 0, 0, particle->radius, particle->position.z, 0, 0, 0, 1;

      // Matrix4f model_view = view * model;

      positions.col(6 * i + 0) << -0.5, -0.5, 0.0, 1.0;
      positions.col(6 * i + 1) <<  0.5, -0.5, 0.0, 1.0;
      positions.col(6 * i + 2) <<  0.5,  0.5, 0.0, 1.0;
      positions.col(6 * i + 3) <<  0.5,  0.5, 0.0, 1.0;
      positions.col(6 * i + 4) << -0.5,  0.5, 0.0, 1.0;
      positions.col(6 * i + 5) << -0.5, -0.5, 0.0, 1.0;

      worldPos.col(6 * i + 0) << particle->position.x, particle->position.y, particle->position.z, 1.0;
      worldPos.col(6 * i + 1) << particle->position.x, particle->position.y, particle->position.z, 1.0;
      worldPos.col(6 * i + 2) << particle->position.x, particle->position.y, particle->position.z, 1.0;
      worldPos.col(6 * i + 3) << particle->position.x, particle->position.y, particle->position.z, 1.0;
      worldPos.col(6 * i + 4) << particle->position.x, particle->position.y, particle->position.z, 1.0;
      worldPos.col(6 * i + 5) << particle->position.x, particle->position.y, particle->position.z, 1.0;

      radii.col(6 * i + 0) << particle->radius;
      radii.col(6 * i + 1) << particle->radius;
      radii.col(6 * i + 2) << particle->radius;
      radii.col(6 * i + 3) << particle->radius;
      radii.col(6 * i + 4) << particle->radius;
      radii.col(6 * i + 5) << particle->radius;

      uvs.col(6 * i + 0) <<  0.0,  0.0;
      uvs.col(6 * i + 1) <<  1.0,  0.0;
      uvs.col(6 * i + 2) <<  1.0,  1.0;
      uvs.col(6 * i + 3) <<  1.0,  1.0;
      uvs.col(6 * i + 4) <<  0.0,  1.0;
      uvs.col(6 * i + 5) <<  0.0,  0.0;

      colourSystem cs =  { "CIE", 0.7355, 0.2645, 0.2658, 0.7243, 0.1669, 0.0085, 0.33333333, 0.33333333, 0 };

      double r, g, b, x, y, z;
      bbTemp = particle->temperature;
      spectrum_to_xyz(bb_spectrum, &x, &y, &z);
      xyz_to_rgb(&cs, x, y, z, &r, &g, &b);

      nanogui::Color col = nanogui::Color(r, g, b, 1.0f);

      if (particle->type == SOOT) {
        col = nanogui::Color(0.05f, 0.05f, 0.05f, 1.0f);
      }

      color.col(6 * i + 0) << col.r(), col.g(), col.b(), 1.0;
      color.col(6 * i + 1) << col.r(), col.g(), col.b(), 1.0;
      color.col(6 * i + 2) << col.r(), col.g(), col.b(), 1.0;
      color.col(6 * i + 3) << col.r(), col.g(), col.b(), 1.0;
      color.col(6 * i + 4) << col.r(), col.g(), col.b(), 1.0;
      color.col(6 * i + 5) << col.r(), col.g(), col.b(), 1.0;

  }
  shader.uploadAttrib("in_position", positions, false);
  shader.uploadAttrib("in_world_pos", worldPos, false);
  shader.uploadAttrib("in_radius", radii, false);
  shader.uploadAttrib("in_uv", uvs, false);
  shader.uploadAttrib("in_color", color, false);

  shader.drawArray(GL_TRIANGLES, 0, particles->size() * 6);

  for (CollisionObject *co : *collision_objects) {
    co->render(shader);
  }

  line_endpoints = vector<Vector3D>();

  for (int i = 0; i < field.width + 2; i++) {
    for (int j = 0; j < field.height + 2; j++) {
      for (int k = 0; k < field.depth + 2; k++) {
        Vector3D cellPos = field.CellPos(i, j, k);
        line_endpoints.push_back(cellPos);
        Vector3D velocity = field.CellAt(field.cells, i, j, k)->velocity;
        line_endpoints.push_back(cellPos + 0.03 * velocity);
      }
    }
  }

  const UserShader& wireframe_shader = shaders[wireframe_shader_idx];
  GLShader &wireframeShader = *wireframe_shader.nanogui_shader;
  wireframeShader.bind();

  Matrix4f model;
  model.setIdentity();
  wireframeShader.setUniform("u_model", model);
  Matrix4f view_projection = projection * view;
  wireframeShader.setUniform("u_view_projection", view_projection);
  wireframeShader.setUniform("u_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f), false);

  if (display_field_velocity) {
    drawLines(wireframeShader);
  }
}

void Simulator::drawLines(GLShader &shader) {
  
  MatrixXf positions(4, line_endpoints.size());

  for (int i = 0; i < line_endpoints.size(); i++) {
    positions.col(i) << line_endpoints[i].x, line_endpoints[i].y, line_endpoints[i].z, 1.0;
  }

  //shader.setUniform("u_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f), false);
  //shader.setUniform("u_color", color, false);
  shader.uploadAttrib("in_position", positions, false);

  shader.drawArray(GL_LINES, 0, line_endpoints.size());
}


// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// OpenGL 3.1 deprecated the fixed pipeline, so we lose a lot of useful OpenGL
// functions that have to be recreated here.
// ----------------------------------------------------------------------------

void Simulator::resetCamera() { camera.copy_placement(canonicalCamera); }

Matrix4f Simulator::getProjectionMatrix() {
  Matrix4f perspective;
  perspective.setZero();

  double cam_near = camera.near_clip();
  double cam_far = camera.far_clip();

  double theta = camera.v_fov() * PI / 360;
  double range = cam_far - cam_near;
  double invtan = 1. / tanf(theta);

  perspective(0, 0) = invtan / camera.aspect_ratio();
  perspective(1, 1) = invtan;
  perspective(2, 2) = -(cam_near + cam_far) / range;
  perspective(3, 2) = -1;
  perspective(2, 3) = -2 * cam_near * cam_far / range;
  perspective(3, 3) = 0;

  return perspective;
}

Matrix4f Simulator::getViewMatrix() {
  Matrix4f lookAt;
  Matrix3f R;

  lookAt.setZero();

  // Convert CGL vectors to Eigen vectors
  // TODO: Find a better way to do this!

  CGL::Vector3D c_pos = camera.position();
  CGL::Vector3D c_udir = camera.up_dir();
  CGL::Vector3D c_target = camera.view_point();

  Vector3f eye(c_pos.x, c_pos.y, c_pos.z);
  Vector3f up(c_udir.x, c_udir.y, c_udir.z);
  Vector3f target(c_target.x, c_target.y, c_target.z);

  R.col(2) = (eye - target).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));

  lookAt.topLeftCorner<3, 3>() = R.transpose();
  lookAt.topRightCorner<3, 1>() = -R.transpose() * eye;
  lookAt(3, 3) = 1.0f;

  return lookAt;
}

// ----------------------------------------------------------------------------
// EVENT HANDLING
// ----------------------------------------------------------------------------

bool Simulator::cursorPosCallbackEvent(double x, double y) {
  if (left_down && !middle_down && !right_down) {
    if (ctrl_down) {
      mouseRightDragged(x, y);
    } else {
      mouseLeftDragged(x, y);
    }
  } else if (!left_down && !middle_down && right_down) {
    mouseRightDragged(x, y);
  } else if (!left_down && !middle_down && !right_down) {
    mouseMoved(x, y);
  }

  mouse_x = x;
  mouse_y = y;

  return true;
}

bool Simulator::mouseButtonCallbackEvent(int button, int action,
                                              int modifiers) {
  switch (action) {
  case GLFW_PRESS:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = true;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = true;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = true;
      break;
    }
    return true;

  case GLFW_RELEASE:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = false;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = false;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = false;
      break;
    }
    return true;
  }

  return false;
}

void Simulator::mouseMoved(double x, double y) { y = screen_h - y; }

void Simulator::mouseLeftDragged(double x, double y) {
  float dx = x - mouse_x;
  float dy = y - mouse_y;

  camera.rotate_by(-dy * (PI / screen_h), -dx * (PI / screen_w));
}

void Simulator::mouseRightDragged(double x, double y) {
  camera.move_by(mouse_x - x, y - mouse_y, canonical_view_distance);
}

bool Simulator::keyCallbackEvent(int key, int scancode, int action,
                                      int mods) {
  ctrl_down = (bool)(mods & GLFW_MOD_CONTROL);

  if (action == GLFW_PRESS) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
      is_alive = false;
      break;
    case 'r':
    case 'R':
      initParticles();
      break;
    case ' ':
      resetCamera();
      break;
    case 'p':
    case 'P':
      is_paused = !is_paused;
      break;
    case 'n':
    case 'N':
      if (is_paused) {
        is_paused = false;
        drawContents();
        is_paused = true;
      }
      break;
    }
  }

  return true;
}

bool Simulator::dropCallbackEvent(int count, const char **filenames) {
  return true;
}

bool Simulator::scrollCallbackEvent(double x, double y) {
  camera.move_forward(y * scroll_rate);
  return true;
}

bool Simulator::resizeCallbackEvent(int width, int height) {
  screen_w = width;
  screen_h = height;

  camera.set_screen_size(screen_w, screen_h);
  return true;
}

void Simulator::initGUI(Screen *screen) {
  Window *window;
  
  window = new Window(screen, "Simulation");
  window->setPosition(Vector2i(default_window_size(0) - 245, 15));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  // Spring types

  new Label(window, "Field", "sans-bold");

  {
    Button *b = new Button(window, "Velocity");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
        [this](bool state) { display_field_velocity = state; });

  }

  // Mass-spring parameters

  new Label(window, "Parameters", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "density :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(1);
    fb->setUnits("g/cm^2");
    fb->setSpinnable(true);
    // fb->setCallback([this](float value) { cp->density = (double)(value * 10); });
  }

  // Simulation constants

  new Label(window, "Simulation", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "frames/s :", "sans-bold");

    IntBox<int> *fsec = new IntBox<int>(panel);
    fsec->setEditable(true);
    fsec->setFixedSize(Vector2i(100, 20));
    fsec->setFontSize(14);
    fsec->setValue(frames_per_sec);
    fsec->setSpinnable(true);
    fsec->setCallback([this](int value) { frames_per_sec = value; });

    new Label(panel, "steps/frame :", "sans-bold");

    IntBox<int> *num_steps = new IntBox<int>(panel);
    num_steps->setEditable(true);
    num_steps->setFixedSize(Vector2i(100, 20));
    num_steps->setFontSize(14);
    num_steps->setValue(simulation_steps);
    num_steps->setSpinnable(true);
    num_steps->setMinValue(0);
    num_steps->setCallback([this](int value) { simulation_steps = value; });
  }

  // Damping slider and textbox

  new Label(window, "Damping", "sans-bold");

  {
    Widget *panel = new Widget(window);
    panel->setLayout(
        new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string(1));
    percentage->setUnits("%");
    percentage->setFontSize(14);
  }

  // Gravity

  new Label(window, "Gravity", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "x :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.x);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.x = value; });

    new Label(panel, "y :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.y);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.y = value; });

    new Label(panel, "z :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(gravity.z);
    fb->setUnits("m/s^2");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { gravity.z = value; });
  }
  
  window = new Window(screen, "Appearance");
  window->setPosition(Vector2i(15, 15));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  // Appearance

  {
    
    
    ComboBox *cb = new ComboBox(window, shaders_combobox_names);
    cb->setFontSize(14);
    cb->setCallback(
        [this, screen](int idx) { active_shader_idx = idx; });
    cb->setSelectedIndex(active_shader_idx);
  }

  // Shader Parameters

  new Label(window, "Color", "sans-bold");

  {
    ColorWheel *cw = new ColorWheel(window, color);
    cw->setColor(this->color);
    cw->setCallback(
        [this](const nanogui::Color &color) { this->color = color; });
  }

  new Label(window, "Parameters", "sans-bold");

  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "Normal :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(this->m_normal_scaling);
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { this->m_normal_scaling = value; });

    new Label(panel, "Height :", "sans-bold");

    fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue(this->m_height_scaling);
    fb->setSpinnable(true);
    fb->setCallback([this](float value) { this->m_height_scaling = value; });
  }
}
