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

  block1.state.pos_x = 20;
  block1.state.pos_y = 10;
  block1.size_x = 8;
  block1.size_y = 2;

  Entity block2;
  block2.shape = box;
  block2.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block2.animations["idle"] = block_idle;
  block2.current_animation = "idle";

  block2.state.pos_x = 0;
  block2.state.pos_y = 0;
  block2.size_x = 2;
  block2.size_y = 2;

  Entity block3;
  block3.shape = box;
  block3.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block3.animations["idle"] = block_idle;
  block3.current_animation = "idle";

  block3.state.pos_x = 10;
  block3.state.pos_y = 6;
  block3.size_x = 10;
  block3.size_y = 2;
  block3.state.angle = 0;

  Entity block4;
  block4.shape = box;
  block4.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block4.animations["idle"] = block_idle;
  block4.current_animation = "idle";

  block4.state.pos_x = 30;
  block4.state.pos_y = 0;
  block4.size_x = 4;
  block4.size_y = 2;

  Entity ball;
  ball.shape = circle;
  ball.load_sprite_sheet(base_path + "..\\resources\\ball.png",
                         graphics.renderer);
                         
  ball.state.pos_x = 0;
  ball.state.pos_y = 6;
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
  block3.name = "block3";
  block4.name = "block4";

  std::vector<Entity*> entities;
  entities.push_back(&ball);
  // entities.push_back(&block1);
  // entities.push_back(&block2);
  entities.push_back(&block3);
  // entities.push_back(&block4);

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
        block3.state.angle += 3;
      if (inputs.is_held(SDL_SCANCODE_PAGEDOWN))
        block3.state.angle -= 3;

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
        if (ball.current_animation == "idle")
          ball.current_animation = "roll";
        else
          ball.current_animation = "idle";
      }
    }

    // ball.state.pos_x += 0.1;

    // graphics.add_to_queue(block1);
    // graphics.add_to_queue(block2);
    graphics.add_to_queue(block3);
    // graphics.add_to_queue(block4);
    
    graphics.add_to_queue(ball);

    // ball.update_state(ball_forces, ball_moments, SDL_GetTicks() - previous_ticks);

    collisions.evaluate_collisions(entities);

    graphics.draw_queue(SDL_GetTicks());

    if (graphics.debug) {
      // Draw origin
      Point origin(0, 0);
      Point right(.5, 0);
      Point left(-.5, 0);
      Point up(0, .5);
      Point down(0, -.5);

      graphics.draw_line(origin, right);
      graphics.draw_line(origin, left);
      graphics.draw_line(origin, up);
      graphics.draw_line(origin, down);
    }

    if (collisions.debug) {
      Point a(0,0);
      Point b(collisions.collision_normal[0], collisions.collision_normal[1]);
      graphics.draw_line(a, b);
    }

    previous_ticks = SDL_GetTicks();
    frameManager.end_frame();
  }
  return 0;
}