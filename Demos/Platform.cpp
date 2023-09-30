#include "../framework.h"

int main(int argc, char** argv) {

  SDL_Init(SDL_INIT_VIDEO);
  std::string base_path = SDL_GetBasePath();

  Graphics graphics;

  Camera camera;
  camera.pos_x = 0;
  camera.pos_y = 0;
  camera.FOV_width = 19.2 * 2;
  camera.FOV_height = 10.8 * 2;

  FrameManager frameManager;
  frameManager.fps = 60;

  InputHandler inputs;

  SDL_Event event;

  // Entities
  Animation block_idle;
  block_idle.add_frame(0, 0, 64, 16);

  Entity block1;
  block1.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block1.animations["idle"] = block_idle;
  block1.current_animation = "idle";

  block1.state.pos_x = 10;
  block1.state.pos_y = 5;
  block1.size_x = 8;
  block1.size_y = 2;

  Entity block2;
  block2.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block2.animations["idle"] = block_idle;
  block2.current_animation = "idle";

  block2.state.pos_x = 0;
  block2.state.pos_y = 0;
  block2.size_x = 5;
  block2.size_y = 5;

  Entity block3;
  block3.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block3.animations["idle"] = block_idle;
  block3.current_animation = "idle";

  block3.state.pos_x = 15;
  block3.state.pos_y = 7;
  block3.size_x = 11;
  block3.size_y = 1;

  Entity block4;
  block4.load_sprite_sheet(base_path + "..\\resources\\block.png",
                           graphics.renderer);
  block4.animations["idle"] = block_idle;
  block4.current_animation = "idle";

  block4.state.pos_x = 30;
  block4.state.pos_y = 0;
  block4.size_x = 4;
  block4.size_y = 2;

  Entity ball;
  ball.load_sprite_sheet(base_path + "..\\resources\\ball.png",
                         graphics.renderer);
                         
  ball.state.pos_x = 0;
  ball.state.pos_y = 0;
  ball.size_x = 2;
  ball.size_y = 2;

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
  ball.current_animation = "roll";

  // Frame Loop
  while (true) {
    frameManager.start_frame();
    inputs.start_frame();
    graphics.clear_screen();
    graphics.clear_queue();

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
      if (inputs.is_held(SDL_SCANCODE_W))
        camera.pos_y += .1;
      if (inputs.is_held(SDL_SCANCODE_A))
        camera.pos_x -= .1;
      if (inputs.is_held(SDL_SCANCODE_S))
        camera.pos_y -= .1;
      if (inputs.is_held(SDL_SCANCODE_D))
        camera.pos_x += .1;

      // Spacebar toggles ball roll
      if (inputs.was_pressed(SDL_SCANCODE_SPACE)) {
        if (ball.current_animation == "idle")
          ball.current_animation = "roll";
        else
          ball.current_animation = "idle";
      }
    }

    graphics.add_to_queue(block1);
    graphics.add_to_queue(block2);
    graphics.add_to_queue(block3);
    graphics.add_to_queue(block4);
    
    graphics.add_to_queue(ball);

    graphics.draw_queue(camera, SDL_GetTicks());
    frameManager.end_frame();
  }

  return 0;
}