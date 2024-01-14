cmake_minimum_required(VERSION 3.10)

# Tablet Paths
#set(SDL2_DIR "C:/Users/johnw/Libraries/SDL2-2.28.3")
#set(SDL2_image_DIR "C:/Users/johnw/Libraries/SDL2_image-2.6.3")
#set(SDL2_ttf_DIR "C:/Libraries/SDL2_ttf-2.20.2")

# PC Paths
set(SDL2_DIR "C:/Libraries/SDL2-2.26.2")
set(SDL2_image_DIR "C:/Libraries/SDL2_image-2.6.2")
#set(SDL2_ttf_DIR "C:/Libraries/SDL2_ttf-2.20.2")

set(SDL2_INCLUDE_DIRS "${SDL2_DIR}/x86_64-w64-mingw32/include/SDL2")
set(SDL2_LIBRARIES "${SDL2_DIR}/x86_64-w64-mingw32/lib/libSDL2main.a;${SDL2_DIR}/x86_64-w64-mingw32/lib/libSDL2.dll.a")

set(SDL2_IMAGE_INCLUDE_DIRS "${SDL2_image_DIR}/x86_64-w64-mingw32/include/SDL2")
set(SDL2_IMAGE_LIBRARIES "${SDL2_image_DIR}/x86_64-w64-mingw32/lib/libSDL2_image.dll.a")

# set(SDL2_TTF_INCLUDE_DIRS "${SDL2_ttf_DIR}/x86_64-w64-mingw32/include/SDL2")
# set(SDL2_TTF_LIBRARIES "${SDL2_ttf_DIR}/x86_64-w64-mingw32/lib/libSDL2_ttf.dll.a")

project(Demo)

add_executable(Demo 
    ../Demo.cpp
)

target_link_libraries(Demo -lmingw32 ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARIES}) # ${SDL2_TTF_LIBRARIES})

target_include_directories(Demo 
    PUBLIC ${SDL2_INCLUDE_DIRS}
    PUBLIC ${SDL2_IMAGE_INCLUDE_DIRS}
    # PUBLIC ${SDL2_TTF_INCLUDE_DIRS}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../
)