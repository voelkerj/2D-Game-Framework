#include "../framework_main.h"
#include <memory>

int main(int argc, char** argv) {

  SDL_Init(SDL_INIT_VIDEO);
  std::string base_path = SDL_GetBasePath();

  Graphics graphics;

  FrameManager frameManager;
  frameManager.fps = 60;

  InputHandler inputs;

  SDL_Event event;

  CollisionProcessor collision_proc;

  // Entities
  Animation block_idle;
  block_idle.add_frame(0, 0, 64, 16);

  // State initial_state;
  std::string sprite_path = base_path + "..\\resources\\block.png";

  // Entity block("test_block", box, initial_state, 4, 4, sprite_path, graphics.renderer);
  // block.animations["idle"] = block_idle;
  // block.current_animation = "idle";

  std::vector<Entity> entities;

  bool print_manifold = false;

  // Frame Loop
  Uint32 previous_ticks{0};
  while (true) {
    frameManager.start_frame();
    inputs.start_frame();
    graphics.clear_screen();
    graphics.clear_queue();

    // collision_proc.evaluate_collisions(entities);
    // collision_proc.prune_resolved_collisions();

    // Handle Keyboard and Close Button
    if (SDL_PollEvent(&event)) {
      // Windows close
      if (event.type == SDL_QUIT)
        return 0;

      // General Keyboard Event
      if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)
        inputs.handle_keyboard_event(event);
      if (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP)
        inputs.handle_mouse_event(event);

      // Escape closes demo
      if (inputs.was_pressed(SDL_SCANCODE_ESCAPE))
        return 0;

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

      if (inputs.left_click) {
        Point mouse_WCS = graphics.convert_screen_coords_to_world_coords(inputs.mouse_coords);

        State new_entity_state;
        new_entity_state.pos_x = mouse_WCS.x;
        new_entity_state.pos_y = mouse_WCS.y;

        // Entity new_entity(box, new_entity_state, 1, 1, sprite_path, graphics.renderer);

        // new_entity.animations["idle"] = block_idle;
        // new_entity.current_animation = "idle";

        entities.emplace_back(box, new_entity_state, 1, 1, sprite_path, graphics.renderer);
        entities.back().animations["idle"] = block_idle;
        entities.back().current_animation = "idle";
      }

      if (inputs.is_held(SDL_SCANCODE_M))
        graphics.current_camera.zoom += 1;
      if (inputs.is_held(SDL_SCANCODE_N))
        graphics.current_camera.zoom -= 1;

      // Debug Outputs
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

      if (inputs.was_pressed(SDL_SCANCODE_C)) {
        std::cout << "Camera: " << graphics.current_camera.pos_x << ", " << graphics.current_camera.pos_y << "\n";
      }

      if (inputs.was_pressed(SDL_SCANCODE_SPACE) && collision_proc.debug) {
        for (int idx = 0; idx < entities.size(); idx++) {
          entities[idx].print_state();
          print_manifold = true;
        }
      }
    }

    if (graphics.debug) {
      // Draw origin
      Point origin(0, 0);
      Point right(.5, 0);
      Point left(-.5, 0);
      Point up(0, .5);
      Point down(0, -.5);

      graphics.add_to_queue(origin, right);
      graphics.add_to_queue(origin, left);
      graphics.add_to_queue(origin, up);
      graphics.add_to_queue(origin, down);
    }

    if (collision_proc.debug) {
      for (auto&& collision : collision_proc.active_collisions_) {
        if (collision->manifold_.size() > 0) {

          if (print_manifold)
            std::cout << "Manifold Points: ";

          for (auto pt : collision->manifold_) {
            if (print_manifold)
              std::cout << "(" << pt.x << ", " << pt.y << "), ";

            graphics.add_to_queue(pt);
          }
          std::cout << "\n";
        }

        // Draw any lines this collison needs drawn
        if (collision->debug_) {
          for (int idx = 0; idx < collision->lines_to_draw_.size(); idx++) {
            graphics.add_to_queue(collision->lines_to_draw_[idx].first,
                                  collision->lines_to_draw_[idx].second);
          }
        }
      }
      if (print_manifold) {
        print_manifold = false;
      }
    }
    
    // Draw all entities
      for (int idx = 0; idx < entities.size(); idx++) {
      // std::cout << "Adding " << entities[idx].name << " to queue.\n";
      graphics.add_to_queue(entities[idx]);
    }

    graphics.draw_queue(SDL_GetTicks());

    previous_ticks = SDL_GetTicks();
    frameManager.end_frame();
  }
  return 0;
}