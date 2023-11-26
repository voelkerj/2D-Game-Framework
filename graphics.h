#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "SDL.h"
#include "entity.h"

#include <vector>

struct Camera {
  // Position of Camera's origin point (center of screen)
  float pos_x;
  float pos_y;

  float zoom{50}; // Camera zoom factor. Zoom factor of 1 means 1 pixel = 1 world unit

  // Width of the Camera's projected FOV
  float FOV_width;
  float FOV_height;
};

class Graphics {
 public:
  std::vector<Entity*> entity_queue;
  std::vector<std::pair<Point, Point>> line_queue;
  std::vector<Vector2> vector_queue;
  std::vector<Point> point_queue;
  SDL_Window* window;
  int window_width;
  int window_height;
  SDL_Renderer* renderer;
  Camera current_camera;
  float scale_x;
  float scale_y;
  bool debug{false};

  Graphics();
  ~Graphics();

  void add_to_queue(Entity& entity);
  void draw_queue(Uint32 current_ticks);
  void clear_queue();

  void add_to_queue(Point a, Point b);
  void add_to_queue(Point a);

  void clear_screen();

 private:
  float debug_grid_size{1};

  void draw_entity(Uint32 current_ticks, Entity* entity);
  void draw_line(Point a, Point b, int color[4]);
  void draw_point(Point a);
  bool point_within_camera_view(float x, float y);
  void draw_grid();
};

Graphics::Graphics() {
  window = SDL_CreateWindow("Framework", SDL_WINDOWPOS_UNDEFINED,
                            SDL_WINDOWPOS_UNDEFINED, 1, 1,
                            SDL_WINDOW_FULLSCREEN_DESKTOP);
  SDL_GetWindowSize(window, &window_width, &window_height);

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF);

  int screen_width, screen_height;
  SDL_GetRendererOutputSize(renderer, &screen_width, &screen_height);

  current_camera.pos_x = 0;
  current_camera.pos_y = 0;
  current_camera.FOV_width = screen_width / current_camera.zoom;
  current_camera.FOV_height = screen_height / current_camera.zoom;

  scale_x = window_width / current_camera.FOV_width;
  scale_y = window_height / current_camera.FOV_height;
}

Graphics::~Graphics() {
  SDL_DestroyWindow(window);
}

void Graphics::add_to_queue(Entity& entity) {
  entity_queue.push_back(&entity);
}

void Graphics::add_to_queue(Point a, Point b) {
  std::pair<Point, Point> line(a, b);
  line_queue.push_back(line);
}

void Graphics::add_to_queue(Point a) {
  point_queue.push_back(a);
}

bool Graphics::point_within_camera_view(float x, float y) {
  // Input a point in world coordinate system, function returns true/false if the camera can see it
  if (x >= (current_camera.pos_x - (current_camera.FOV_width / 2)) &
      x <= (current_camera.pos_x + (current_camera.FOV_width / 2)) &
      y >= (current_camera.pos_y - (current_camera.FOV_height / 2)) &
      y <= (current_camera.pos_y + (current_camera.FOV_height / 2)))
    return true;

  return false;
}

void Graphics::draw_queue(Uint32 current_ticks) {
  //Update camera FOV
  int screen_width, screen_height;
  SDL_GetRendererOutputSize(renderer, &screen_width, &screen_height);
  current_camera.FOV_width = screen_width / current_camera.zoom;
  current_camera.FOV_height = screen_height / current_camera.zoom;

  scale_x = window_width / current_camera.FOV_width;
  scale_y = window_height / current_camera.FOV_height;

  if (debug) {
    draw_grid();
  }

  // Draw Entities
  for (auto& entity : entity_queue) {
    draw_entity(current_ticks, entity);
  }

  // Draw Lines
  for (auto line : line_queue) {
    int color[4] = {0,255,0,255};
    draw_line(line.first, line.second, color);
  }

  // Draw Points
  for (auto point : point_queue) {
    draw_point(point);
  }

  SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF);
  SDL_RenderPresent(renderer);
}

void Graphics::draw_entity(Uint32 current_ticks, Entity* entity) {
  // If entity is within the camera bounds
  entity->calculate_vertices();
  entity->calculate_vertices_in_WCS();

  bool inFOV{false};

  for (Point pt : entity->vertices_WCS) {
    if (point_within_camera_view(pt.x, pt.y))
      inFOV = true;
  }

    if (inFOV) {
    // Draw Entity
    Animation& animation = entity->animations[entity->current_animation];

    SDL_Rect clipping_rect = animation.get_frame(current_ticks);

    SDL_Rect destination_rect;
    destination_rect.x = (entity->state.pos_x - entity->size_x / 2 - (current_camera.pos_x - (current_camera.FOV_width / 2))) * scale_x;
    destination_rect.y = window_height - (entity->state.pos_y - entity->size_y / 2 - (current_camera.pos_y - (current_camera.FOV_height / 2))) * scale_y - (entity->size_y * scale_y);
    destination_rect.w = entity->size_x * scale_x;
    destination_rect.h = entity->size_y * scale_y;

    SDL_RenderCopyEx(renderer, entity->sprite_sheet, &clipping_rect,
                     &destination_rect, -entity->state.angle, NULL,
                     SDL_FLIP_NONE);

    // Draw Vertices
    if (debug) {
      // Color entities red when they collide
      // if (entity->collision) {
      //   SDL_SetTextureColorMod(entity->sprite_sheet, 255, 0, 0);
      // } else {
      //   SDL_SetTextureColorMod(entity->sprite_sheet, 255, 255, 255);
      // }

      // Display entity vertices
      for (Point point : entity->vertices_WCS) {
        SDL_SetRenderDrawColor(renderer, 0x48, 0xff, 0x82, 0xFF);
        float pt_x = (point.x - (current_camera.pos_x - (current_camera.FOV_width / 2))) * scale_x;
        float pt_y = window_height - (point.y - entity->size_y - (current_camera.pos_y - (current_camera.FOV_height / 2))) * scale_y - (entity->size_y * scale_y);

        SDL_RenderDrawPoint(renderer, pt_x, pt_y);
      }

      SDL_SetRenderDrawColor(renderer, 0x26, 0x26, 0x26, 0xFF);
    }
  }
}

void Graphics::draw_line(Point a, Point b, int color[4]) {
  float a_x = (a.x - (current_camera.pos_x - (current_camera.FOV_width / 2))) * scale_x;
  float a_y = window_height - (a.y - (current_camera.pos_y - (current_camera.FOV_height / 2))) * scale_y;

  float b_x = (b.x - (current_camera.pos_x - (current_camera.FOV_width / 2))) * scale_x;
  float b_y = window_height - (b.y - (current_camera.pos_y - (current_camera.FOV_height / 2))) * scale_y;

  SDL_SetRenderDrawColor(renderer, color[0], color[1], color[2], color[3]);

  SDL_RenderDrawLine(renderer, a_x, a_y, b_x, b_y);
}

void Graphics::draw_point(Point a) {
  SDL_SetRenderDrawColor(renderer, 0xff, 0xe9, 0x1a, 0xFF);

  float a_x = (a.x - (current_camera.pos_x - (current_camera.FOV_width / 2))) * scale_x;
  float a_y = window_height - (a.y - (current_camera.pos_y - (current_camera.FOV_height / 2))) * scale_y;

  SDL_RenderDrawPoint(renderer, a_x, a_y);  // Actual Point

  SDL_RenderDrawPoint(renderer, a_x + 1, a_y);
  SDL_RenderDrawPoint(renderer, a_x - 1, a_y);
  SDL_RenderDrawPoint(renderer, a_x, a_y + 1);
  SDL_RenderDrawPoint(renderer, a_x, a_y - 1);
  SDL_RenderDrawPoint(renderer, a_x + .7071, a_y + .7071);
  SDL_RenderDrawPoint(renderer, a_x + .7071, a_y - .7071);
  SDL_RenderDrawPoint(renderer, a_x - .7071, a_y - .7071);
  SDL_RenderDrawPoint(renderer, a_x - .7071, a_y + .7071);
}

void Graphics::clear_queue() {
  entity_queue.clear();
  point_queue.clear();
  line_queue.clear();
  vector_queue.clear();
}

void Graphics::clear_screen() {
  SDL_RenderClear(renderer);
}

void Graphics::draw_grid() {
  int color[4] = {15, 15, 15, 255};

  // Get x/y min/max of grid lines
  int lines_xmin = floor((current_camera.pos_x - (current_camera.FOV_width / 2)));
  int lines_xmax = ceil((current_camera.pos_x - (current_camera.FOV_width / 2)) + current_camera.FOV_width);
  int lines_ymin = floor((current_camera.pos_y - (current_camera.FOV_height / 2)));
  int lines_ymax = ceil((current_camera.pos_y - (current_camera.FOV_height / 2)) + current_camera.FOV_height);

  // Queue up vertical lines
  int num_vertical_lines = lines_xmax - lines_xmin;
  for (int idx = 0; idx < num_vertical_lines; idx++) {
    Point start(lines_xmin + (idx * debug_grid_size), lines_ymin);
    Point end(lines_xmin + (idx * debug_grid_size), lines_ymax);
    draw_line(start, end, color);
  }

  // Queue up horizontal lines
  int num_horizontal_lines = lines_ymax - lines_ymin;
  for (int idx = 0; idx < num_horizontal_lines; idx++) {
    Point start(lines_xmin, lines_ymin + (idx * debug_grid_size));
    Point end(lines_xmax, lines_ymin + (idx * debug_grid_size));
    draw_line(start, end, color);
  }
}

#endif