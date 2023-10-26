#include "../framework.h"

int main(int argc, char** argv) {

  SDL_Init(SDL_INIT_VIDEO);
  std::string base_path = SDL_GetBasePath();

  Graphics graphics;

  FrameManager frameManager;
  frameManager.fps = 60;

  InputHandler inputs;

  SDL_Event event;

  CollisionProcessor collisions;

  // Entities
  Animation block_idle;
  block_idle.add_frame(0, 0, 64, 16);

  Entity block1;
  block1.shape = box;
  block1.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block1.animations["idle"] = block_idle;
  block1.current_animation = "idle";

  // block1.vertices_WCS.push_back()

  // Example 1
  // block1.state.pos_x = 11;
  // block1.state.pos_y = 6.5;
  // block1.size_x = 6;
  // block1.size_y = 5;

  // Example 2
  // block1.state.pos_x = 5.5;
  // block1.state.pos_y = 7.5;
  // block1.size_x = sqrt(18);
  // block1.size_y = sqrt(32);
  // block1.state.angle = 45;

  // Example 3
  block1.state.pos_x = 11.5;
  block1.state.pos_y = 5.5;
  block1.size_x = sqrt(17);
  block1.size_y = sqrt(17);
  block1.state.angle = -22.3082224007915 - 3;

  Entity block2;
  block2.shape = box;
  block2.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block2.animations["idle"] = block_idle;
  block2.current_animation = "idle";

  block2.state.pos_x = 8;
  block2.state.pos_y = 3.5;
  block2.size_x = 8;
  block2.size_y = 3;

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

  ball.name = "ball";
  block1.name = "block1";
  block2.name = "block2";

  std::vector<Entity*> entities;
  entities.push_back(&ball);
  entities.push_back(&block1);
  // entities.push_back(&block2);

  // Frame Loop
  Uint32 previous_ticks{0};
  while (true) {
    frameManager.start_frame();
    inputs.start_frame();
    graphics.clear_screen();
    graphics.clear_queue();
    
    ball_forces.clear();
    ball_moments.clear();

    collisions.reset_collisions(entities);

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
        ball.state.pos_y += 0.1;
        // block2.state.pos_y += 0.1;
      }
      if (inputs.is_held(SDL_SCANCODE_A)){
        ball.state.pos_x -= 0.1;
        // block2.state.pos_x -= 0.1;
      }
      if (inputs.is_held(SDL_SCANCODE_S)){
        ball.state.pos_y -= 0.1;
        // block2.state.pos_y -= 0.1;
      }
      if (inputs.is_held(SDL_SCANCODE_D)){
        ball.state.pos_x += 0.1;
        // block2.state.pos_x += 0.1;
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

      if (inputs.is_held(SDL_SCANCODE_PAGEUP))
        block1.state.angle += 3;
      if (inputs.is_held(SDL_SCANCODE_PAGEDOWN))
        block1.state.angle -= 3;

      if (inputs.was_pressed(SDL_SCANCODE_F1)) {
        if (collisions.debug)
          collisions.debug = false;
        else
          collisions.debug = true;

        if (graphics.debug)
          graphics.debug = false;
        else
          graphics.debug = true;
      }

      if (inputs.was_pressed(SDL_SCANCODE_SPACE)) {
        std::cout << "Ball Position: " << ball.state.pos_x << ", " << ball.state.pos_y << "\n";
        std::cout << "Block Angle: " << block1.state.angle << " deg\n";
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

    if (collisions.debug) {
      collisions.evaluate_collisions(entities);
      if (collisions._manifold.size() > 0) {
        for (auto pt : collisions._manifold) {
          graphics.add_to_queue(pt);

          Point b(collisions.collision_normal[0] + pt.x,
                  collisions.collision_normal[1] + pt.y);
          graphics.add_to_queue(pt, b);
        }
      }
    }

    graphics.add_to_queue(block1);
    // graphics.add_to_queue(block2);
    graphics.add_to_queue(ball);
    
    graphics.draw_queue(SDL_GetTicks());

    previous_ticks = SDL_GetTicks();
    frameManager.end_frame();
  }
  return 0;
}