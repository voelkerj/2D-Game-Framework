#include "../framework_main.h"

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

  Entity block1;
  block1.shape = box;
  block1.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block1.animations["idle"] = block_idle;
  block1.current_animation = "idle";

  block1.state.pos_x = 8;
  block1.state.pos_y = 3.5;
  block1.size_x = 8;
  block1.size_y = 3;

  // Example 2
  // block1.state.pos_x = 5.5;
  // block1.state.pos_y = 7.5;
  // block1.size_x = sqrt(18);
  // block1.size_y = sqrt(32);
  // block1.state.angle = 45;

  // Example 3
  // block1.state.pos_x = 11.5;
  // block1.state.pos_y = 5.5;
  // block1.size_x = sqrt(17);
  // block1.size_y = sqrt(17);
  // block1.state.angle = -22.3082224007915;

  Entity ball;
  ball.shape = circle;
  ball.load_sprite_sheet(base_path + "..\\resources\\ball.png",
                         graphics.renderer);
                         
  ball.state.pos_x = 8.4;
  ball.state.pos_y = 5.6;
  ball.size_x = 2;
  ball.size_y = 2;

  ball.mass = 5;

  Animation ball_roll;
  ball_roll.add_frame(0, 0, 16, 16);
  ball_roll.add_frame(16, 0, 16, 16);
  ball_roll.add_frame(32, 0, 16, 16);
  ball_roll.add_frame(48, 0, 16, 16);
  ball_roll.add_frame(64, 0, 16, 16);
  ball_roll.add_frame(80, 0, 16, 16);
  ball_roll.add_frame(96, 0, 16, 16);
  ball_roll.add_frame(112, 0, 16, 16);
  ball_roll.update_interval = 50;

  Animation ball_idle;
  ball_idle.add_frame(0, 0, 16, 16);

  ball.animations["roll"] = ball_roll;
  ball.animations["idle"] = ball_idle;
  ball.current_animation = "idle";
  
  std::vector<Force> ball_forces;

  std::vector<Moment> ball_moments;

  Entity movable_box;
  movable_box.shape = box;
  movable_box.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  movable_box.animations["idle"] = block_idle;
  movable_box.current_animation = "idle";

  movable_box.state.pos_x = 0;
  movable_box.state.pos_y = 0;
  movable_box.size_x = sqrt(17);
  movable_box.size_y = sqrt(17);

  ball.name = "ball";
  block1.name = "block1";
  movable_box.name = "movable";

  std::vector<Entity*> entities;
  // entities.push_back(&ball);
  entities.push_back(&movable_box);
  entities.push_back(&block1);
  // entities.push_back(&block2);
  
  bool print_manifold = false;

  // Frame Loop
  Uint32 previous_ticks{0};
  while (true) {
    frameManager.start_frame();
    inputs.start_frame();
    graphics.clear_screen();
    graphics.clear_queue();
    
    ball_forces.clear();
    ball_moments.clear();

    // Handle Keyboard and Close Button
    if (SDL_PollEvent(&event)) {
      // Windows close
      if (event.type == SDL_QUIT)
        return 0;

      // General Keyboard Event
      if (event.type == SDL_KEYDOWN || SDL_KEYUP)
        inputs.handle_keyboard_event(event);

      // Escape closes demo
      if (inputs.was_pressed(SDL_SCANCODE_ESCAPE))
        return 0;

      // WASD Moves Camera
      if (inputs.is_held(SDL_SCANCODE_W)){
        movable_box.state.pos_y += 0.1;
      }
      if (inputs.is_held(SDL_SCANCODE_A)){
        movable_box.state.pos_x -= 0.1;
      }
      if (inputs.is_held(SDL_SCANCODE_S)){
        movable_box.state.pos_y -= 0.1;
      }
      if (inputs.is_held(SDL_SCANCODE_D)){
        movable_box.state.pos_x += 0.1;
      }

      if (inputs.is_held(SDL_SCANCODE_UP) || inputs.was_pressed(SDL_SCANCODE_UP)){
        graphics.current_camera.pos_y += 0.2;
      }
      if (inputs.is_held(SDL_SCANCODE_DOWN) || inputs.was_pressed(SDL_SCANCODE_DOWN)){
        graphics.current_camera.pos_y -= 0.2;
      }
      if (inputs.is_held(SDL_SCANCODE_LEFT) || inputs.was_pressed(SDL_SCANCODE_LEFT)){
        graphics.current_camera.pos_x -= 0.2;
      }
      if (inputs.is_held(SDL_SCANCODE_RIGHT) || inputs.was_pressed(SDL_SCANCODE_RIGHT)){
        graphics.current_camera.pos_x += 0.2;
      }

      if (inputs.was_pressed(SDL_SCANCODE_F2)) {
        movable_box.state.pos_x = 4;
        movable_box.state.pos_y = 0;
        movable_box.state.angle = -24;
      }

      if (inputs.is_held(SDL_SCANCODE_PAGEUP))
        movable_box.state.angle += 3;
      if (inputs.is_held(SDL_SCANCODE_PAGEDOWN))
        movable_box.state.angle -= 3;

      if (inputs.is_held(SDL_SCANCODE_Q))
        block1.state.angle += 3;
      if (inputs.is_held(SDL_SCANCODE_E))
        block1.state.angle -= 3;

      if (inputs.was_pressed(SDL_SCANCODE_F1)) {
        if (collision_proc.debug) // Hardcoded to be true above
          collision_proc.debug = false;
        else
          collision_proc.debug = true;

        if (graphics.debug)
          graphics.debug = false;
        else
          graphics.debug = true;
      }

      if (inputs.was_pressed(SDL_SCANCODE_SPACE)) {
        for (auto entity : entities) {
          entity->print_state();
          print_manifold = true;
        }

      //   if (ball.current_animation == "idle")
      //     ball.current_animation = "roll";
      //   else
      //     ball.current_animation = "idle";
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
      collision_proc.evaluate_collisions(entities);
      for (auto&& collision : collision_proc.active_collisions_) 
      {
        if (collision->manifold_.size() > 0) {
          
          if (print_manifold)
            std::cout << "Manifold Points: ";

          for (auto pt : collision->manifold_) {
            if (print_manifold)
              std::cout << "(" << pt.x << ", " << pt.y << "), ";

            graphics.add_to_queue(pt);

          }

          if (print_manifold) {
            std::cout << "\n";
            print_manifold = false;
          }
        }

        // Draw any lines this collison needs drawn
        if (collision->debug_) {
          for (int idx = 0; idx < collision->lines_to_draw_.size(); idx ++) {
            graphics.add_to_queue(collision->lines_to_draw_[idx].first, collision->lines_to_draw_[idx].second);
          }
        }
      }
      collision_proc.prune_resolved_collisions();
    }

    // Point test1(5,5);
    // Point test2(6,5);
    // Point test3(5,6);
    // graphics.add_to_queue(test1);
    // graphics.add_to_queue(test2);
    // graphics.add_to_queue(test3);

    graphics.add_to_queue(movable_box);
    graphics.add_to_queue(block1);
    // graphics.add_to_queue(ball);
    
    graphics.draw_queue(SDL_GetTicks());

    previous_ticks = SDL_GetTicks();
    frameManager.end_frame();
  }
  return 0;
}