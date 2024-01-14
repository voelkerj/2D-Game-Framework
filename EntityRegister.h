#ifndef ENTITY_REGISTER_H
#define ENTITY_REGISTER_H

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

class Entity;

// Entity Processor class
class EntityRegister {
  public:
    std::vector<Entity*> Entities;

    void register_entity(Entity& entity);
    bool check_in_registry(Entity& entity);
    void unregister_entity(Entity& entity);

    void print_registry();
};

void EntityRegister::register_entity(Entity& entity) {
  Entity* ptr = &entity;
  Entities.push_back(ptr);
}

bool EntityRegister::check_in_registry(Entity& entity) {
  auto it = std::find(Entities.begin(), Entities.end(), &entity);
  if (it != Entities.end())
    return true;
  else
    return false;
}

void EntityRegister::unregister_entity(Entity& entity) {
  auto it = std::find(Entities.begin(), Entities.end(), &entity);
  if (it != Entities.end())
    Entities.erase(it);
}

void EntityRegister::print_registry() {
  for (int idx = 0; idx < Entities.size(); idx++)
    std::cout << Entities[idx] << "\n";
}

#endif