#pragma once
#include <string>
#include <unordered_map>

struct OntologyItem {
  bool dangerous{false};
  bool fragile{false};
};

class Ontology {
public:
  bool load(const std::string &path);
  const OntologyItem* get(const std::string &name) const;

private:
  std::unordered_map<std::string, OntologyItem> items_;
};

