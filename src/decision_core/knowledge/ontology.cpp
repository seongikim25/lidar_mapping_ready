#include "ontology.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

bool Ontology::load(const std::string &path)
{
  YAML::Node root = YAML::LoadFile(path);

  for (auto it : root) {
    OntologyItem item;
    item.dangerous = it.second["dangerous"].as<bool>(false);
    item.fragile   = it.second["fragile"].as<bool>(false);
    items_[it.first.as<std::string>()] = item;
  }
  return true;
}

const OntologyItem* Ontology::get(const std::string &name) const
{
  auto it = items_.find(name);
  if (it == items_.end()) return nullptr;
  return &it->second;
}

