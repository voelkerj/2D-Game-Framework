#include "../framework_main.h"

#include <list>

int main(int argc, char** argv) {

  // Start SDL
  SDL_Init(SDL_INIT_VIDEO);
  std::string base_path = SDL_GetBasePath();
  SDL_Event event;

  // Initialize Framework Governing Classes
  FrameManager frameManager;
  Graphics graphics;
  graphics.current_camera.zoom = 10;
  CollisionProcessor collision_proc;
  InputHandler inputs;

  // Set up block animation and sprite path
  Animation block_idle;
  block_idle.add_frame(0, 0, 64, 16);
  std::string sprite_path = base_path + "..\\resources\\block.png";

  // Set up entities list and iterator
  std::list<Entity> entities;
  std::list<Entity>::iterator entity;

  // Gravity
  Vector2 gravity(0,-1);

  // Floor
  State floor_state;
  floor_state.pos_x = 0;
  floor_state.pos_y = -30;
  Entity floor("floor", box, floor_state, 100, 5, sprite_path, graphics.renderer);
  floor.animations["idle"] = block_idle;
  floor.current_animation = "idle";
  entities.push_back(floor);

  // Frame Loop
  Uint32 previous_ticks{0};

  while (true) {
    // Start Frame
    frameManager.start_frame();
    inputs.start_frame();
    graphics.start_frame();

    // Handle Events
    if (SDL_PollEvent(&event)) {

      // Windows close
      if (event.type == SDL_QUIT)
        break;

      // General Keyboard/Mouse Event
      if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)
        inputs.handle_keyboard_event(event);
      if (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP)
        inputs.handle_mouse_event(event);

      // Escape closes demo
      if (inputs.was_pressed(SDL_SCANCODE_ESCAPE))
        break;
      
      // Move Camera
      if (inputs.is_held(SDL_SCANCODE_UP) || inputs.was_pressed(SDL_SCANCODE_UP)) {
        graphics.current_camera.pos_y += 0.2;
      }
      if (inputs.is_held(SDL_SCANCODE_DOWN) || inputs.was_pressed(SDL_SCANCODE_DOWN)) {
        graphics.current_camera.pos_y -= 0.2;
      }
      if (inputs.is_held(SDL_SCANCODE_LEFT) || inputs.was_pressed(SDL_SCANCODE_LEFT)) {
        graphics.current_camera.pos_x -= 0.2;
      }
      if (inputs.is_held(SDL_SCANCODE_RIGHT) || inputs.was_pressed(SDL_SCANCODE_RIGHT)) {
        graphics.current_camera.pos_x += 0.2;
      }

      // Zoom camera
      if (inputs.is_held(SDL_SCANCODE_M))
        graphics.current_camera.zoom += 1;
      if (inputs.is_held(SDL_SCANCODE_N))
        graphics.current_camera.zoom -= 1;

      // Left click spawns new entity
      if (inputs.left_click) {
        Point mouse_WCS = graphics.convert_screen_coords_to_world_coords(inputs.mouse_coords);
 
        State new_entity_state;
        new_entity_state.pos_x = mouse_WCS.x;
        new_entity_state.pos_y = mouse_WCS.y;

        Entity new_entity(box, new_entity_state, 1, 1, sprite_path, graphics.renderer);

        new_entity.mass = 1;
        new_entity.update_acceleration(gravity);

        new_entity.animations["idle"] = block_idle;
        new_entity.current_animation = "idle";

        entities.push_back(new_entity);
      }

      // F1 toggles debug states
      if (inputs.was_pressed(SDL_SCANCODE_F1)) {
        if (collision_proc.debug)
          collision_proc.debug = false;
        else
          collision_proc.debug = true;

        if (graphics.debug)
          graphics.debug = false;
        else
          graphics.debug = true;
      }
    }

    // Update Entity Velocities
    for (entity = entities.begin(); entity != entities.end(); ++entity) {
      entity->update_velocity(frameManager.elapsed_time);
    }

    // Add all entities to collison queue (if not already in queue)
    for (entity = entities.begin(); entity != entities.end(); ++entity) {
      if (!collision_proc.entities.check_in_registry(*entity))
        collision_proc.entities.register_entity(*entity);
    }

    // Evaluate collisions
    collision_proc.evaluate_collisions(frameManager.elapsed_time);
    collision_proc.prune_resolved_collisions();

    // Add collision debug elements to drawing queue
    if (collision_proc.debug) {
      for (int idx = 0; idx < collision_proc.active_collisions_.size(); idx++) {
        if (collision_proc.active_collisions_[idx].manifold_.size() > 0) {

          for (auto pt : collision_proc.active_collisions_[idx].manifold_) {
            graphics.add_to_queue(pt);
          }
        }
      }
    }

    // Update Entity Positions
    for (entity = entities.begin(); entity != entities.end(); ++entity) {
      entity->update_position(frameManager.elapsed_time);
      entity->calculate_vertices();
      entity->calculate_vertices_in_WCS();
    }

    // Add all entities to drawing queue
    for (entity = entities.begin(); entity != entities.end(); ++entity) {
      graphics.add_to_queue(*entity);
    }

    // Draw queue
    graphics.draw_queue(SDL_GetTicks());

    // End frame
    frameManager.end_frame();
  }
  return 0;
}