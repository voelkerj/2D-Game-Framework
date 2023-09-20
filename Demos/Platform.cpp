#include "../framework.h"

int main(int argc, char **argv) {

  SDL_Init(SDL_INIT_VIDEO);

  Graphics graphics;

  World world;
  world.size_x = 100;
  world.size_y = 100;

  Camera camera;
  camera.pos_x = 0;
  camera.pos_y = 0;
  camera.FOV_width = 19.2;
  camera.FOV_height = 10.8;

  FrameManager frameManager;
  frameManager.fps = 60;

  InputHandler inputs;

  SDL_Event event;

  // Entities
  // Entity ball;
  // ball.load_sprite_sheet("ball.png", graphics.renderer);

  // Animation roll;
  // roll.add_frame(0, 0, 16, 16);
  // roll.add_frame(16, 0, 16, 16);
  // roll.add_frame(0, 16, 16, 16);
  // roll.add_frame(16, 16, 16, 16);

  // Animation idle;
  // idle.add_frame(0, 0, 16, 16);

  // ball.animations["roll"] = roll;
  // ball.animations["idle"] = idle;

  Entity block;
  block.load_sprite_sheet("block.png", graphics.renderer);

  Animation block_idle;
  block_idle.add_frame(0,0,64,16);
  block.animations["idle"] = block_idle;
  block.current_animation = "idle";

  block.state.pos_x = 5;
  block.state.pos_y = 5;
  block.size_x = 8;
  block.size_y = 4;

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
      if (inputs.was_pressed(SDL_SCANCODE_ESCAPE) == true)
        return 0; 

      // WASD Moves Camera
      if (inputs.is_held(SDL_SCANCODE_W) == true)
        camera.pos_y += 1;
      if (inputs.is_held(SDL_SCANCODE_A) == true)
        camera.pos_x -= 1;
      if (inputs.is_held(SDL_SCANCODE_S) == true)
        camera.pos_y -= 1;
      if (inputs.is_held(SDL_SCANCODE_D) == true)
        camera.pos_x += 1;

    }

    // std::cout << "Camera (" << camera.pos_x << ", " << camera.pos_y << ")\n";

    graphics.add_to_queue(block);

    graphics.draw_queue(camera, SDL_GetTicks());
    frameManager.end_frame();
  }

  return 0;
}