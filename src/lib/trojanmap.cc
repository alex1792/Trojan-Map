#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Students should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return
 * -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string &id) { 
  return data[id].lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist,
 * return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string &id) {
  return data[id].lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return
 * "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string &id) {
  return data[id].name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return
 * an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id) {
  std::vector<std::string> nb_ids;
  for(std::string id : data[id].neighbors) {
    nb_ids.push_back(id);
  }
  return nb_ids;
}

/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 * The location name must be unique, which means there is only one node with the name.
 *
 * @param  {std::string} name          : location name
 * @return {std::string}               : id
 */
std::string TrojanMap::GetID(const std::string &name) {
  std::string res = "";
  for(auto &p : data) {
    if(p.second.name == name) {
      return p.first;
    }
  }
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not
 * exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);

  for(auto &p : data) {
    if(p.second.name == name) {
      results.first = p.second.lat;
      results.second = p.second.lon;
      break;
    }
  }

  return results;
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * @param  {std::string} a          : first string
 * @param  {std::string} b          : second string
 * @return {int}                    : edit distance between two strings
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b) {     
  int m = a.size();
  int n = b.size();

  // to lowercase
  std::transform(a.begin(), a.end(), a.begin(), ::tolower);
  std::transform(b.begin(), b.end(), b.begin(), ::tolower);


  // create dp table
  std::vector<std::vector<int>> dp(m + 1, std::vector<int>(n + 1, 0));

  // initialize dp table
  for(int i = 0; i <= m; i++)
    dp[i][0] = i;
  for(int j = 0; j <= n; j++)
    dp[0][j] = j;

  // dp to find the number of edit
  for(int i = 1; i <= m; i++) {
    for(int j = 1; j <= n; j++) {
      const int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
      // dp[i - 1][j + 1: delete one character
      // dp[i][j - 1] + 1: insert one character
      // dp[i- 1][j - 1] + cost: replace one charactor
      dp[i][j] = std::min({dp[i - 1][j] + 1, dp[i][j - 1] + 1, dp[i - 1][j - 1] + cost});
    }
  }
  
  return dp[m][n];
}

/**
 * FindClosestName: Given a location name, return the name with the smallest edit
 * distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : the closest name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = ""; // Start with a dummy word
  int lowest_cost = name.size();

  for(auto &p : data) {
    // names will be transformed into lower case in CalculateEditDistance function
    int dist = CalculateEditDistance(name, p.second.name);
    if(dist <= lowest_cost) {
      tmp = p.second.name;
      lowest_cost = dist;
    }
  } 
  
  
  return tmp;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
  
  // empty name --> return empty results
  if(name.size() == 0)  return results;
  
  // A map of ids to Nodes.
  // std::unordered_map<std::string, Node> data;

  // transform into lowercase characters
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);

  // traverse all Nodes and compare with its name
  for(auto &p : data) {
    // to lowercase
    std::string node_name = p.second.name;
    std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);
    
    // check prefix
    if(node_name.find(name) == 0) {
      // prefix found in node name
      results.push_back(p.second.name);
    }
  }

  return results;
}

/**
 * GetAllCategories: Return all the possible unique location categories, i.e.
 * there should be no duplicates in the output.
 *
 * @return {std::vector<std::string>}  : all unique location categories
 */
std::vector<std::string> TrojanMap::GetAllCategories() {
  // std::unordered_set<std::string> cats;  
  std::set<std::string> cats;
  
  // traverse each node in data, and insert the categories into set cats
  for(auto &p : data) {
    for(auto &c : p.second.attributes) {
      cats.insert(c);
    }
  }

  // transform the set into vector
  std::vector<std::string> result;
  for(auto &c : cats)
    result.push_back(c);

  return result;
}

/**
 * GetAllLocationsFromCategory: Return all the locations of the input category (i.e.
 * 'attributes' in data.csv). If there is no location of that category, return
 * (-1, -1). The function should be case-insensitive.
 *
 * @param  {std::string} category         : category name (attribute)
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetAllLocationsFromCategory(
    std::string category) {
  std::vector<std::string> res;
  for(auto &d : data) {
    if(d.second.attributes.find(category) != d.second.attributes.end()) {
      res.push_back(d.first);
    }
  }

  return res;
}

/**
 * GetLocationRegex: Given the regular expression of a location's name, your
 * program should first check whether the regular expression is valid, and if so
 * it returns all locations that match that regular expression.
 *
 * @param  {std::regex} location name      : the regular expression of location
 * names
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetLocationRegex(std::regex location) {
  std::vector<std::string> ret;

  for(const auto &p : data) {
    if(std::regex_match(p.second.name, location)) {
      ret.push_back(p.first);
    }
  }

  return ret;
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 * We have provided the code for you. Please do not need to change this function.
 * You can use this function to calculate the distance between 2 nodes.
 * The distance is in mile.
 * The distance is calculated using the Haversine formula.
 * https://en.wikipedia.org/wiki/Haversine_formula
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id,
                                    const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) *
                                           cos(b.lat * M_PI / 180.0) *
                                           pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations
 * inside the vector.
 * We have provided the code for you. Please do not need to change this function.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++) {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path
 * which is a list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  // get start_id and end_id
  std::string start_id = GetID(location1_name);
  std::string end_id = GetID(location2_name);

  // init essential data structures(tables to record distance, predecessors, and priority queue)
  std::unordered_map<std::string, double> dist;       //  <id, distance>
  std::unordered_map<std::string, std::string> prev;  // <id, prev_id>
  // pair<distance, node_id>
  // priority_queue<pair, vector<pair>> pq
  std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<std::pair<double, std::string>>> pq;

  // initialize distance to infinity
  for(const auto &p : data)
      dist[p.first] = DBL_MAX;

  // set distance of starting point to 0 and push it into priority queue
  dist[start_id] = 0;
  pq.push({0, start_id});

  // keep accessing neighbors of pq.top till reach end or pq.is_empty
  while(!pq.empty()) {
    // current distance & current id of pq.top
    double current_dist = pq.top().first;
    std::string current_id = pq.top().second;
    pq.pop();

    // check if reach destination
    if(current_id == end_id)
      break;
    
    // check distance is shorter or not
    if(current_dist > dist[current_id])
      continue;
    
    // processing neighbors
    for(auto &nb_id : data[current_id].neighbors) {
      double new_dist = current_dist + CalculateDistance(current_id, nb_id);

      // update shortest distance
      if(new_dist < dist[nb_id]) {
        dist[nb_id] = new_dist;
        prev[nb_id] = current_id;
        pq.push({new_dist, nb_id});
      }
    }
  }

  // transform prev map to path vector
  std::vector<std::string> path;
  std::string current_id = end_id;
  while(current_id != start_id) {
    path.push_back(current_id);
    
    // check if path does not exist
    if(prev.find(current_id) == prev.end())
      return {};
    
    // update current_id
    current_id = prev[current_id];
  }

  // push start into path
  path.push_back(start_id);

  // reverse path since we start from end, path is in opposite direcion
  std::reverse(path.begin(), path.end());

  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest
 * path which is a list of id. Hint: Do the early termination when there is no
 * change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name) {
  // get start_id and end_id
  std::string start_id = GetID(location1_name);
  std::string end_id = GetID(location2_name);
  if(start_id.empty() || end_id.empty())
      return {};

  // initialize distance
  std::unordered_map<std::string, double> dist;         //  <id, distance>
  std::unordered_map<std::string, std::string> prev;    //  <current_id, prev_id>
  for(auto &p : data)
      dist[p.first] = DBL_MAX;
  dist[start_id] = 0;

  // go throught # of nodes - 1 times
  int node_cnt = data.size();
  // std::cout << node_cnt << std::endl;
  for(int i = 0; i < node_cnt - 1; i++) {
    // visit all nodes
    // has_updated is used for early stopping since the number of node is very large
    bool has_updated = false;
    for(const auto &p : data) {
      std::string cur_id = p.first;
      
      // visit all neighbors of current node
      for(const auto &nb_id : p.second.neighbors) {
        // calculate the distance from current node to neighbor node
        double weight = CalculateDistance(cur_id, nb_id);
        
        // update distance and predecessor when distance to neighbor is shorter
        if(dist[cur_id] != DBL_MAX && dist[cur_id] + weight < dist[nb_id]) {
          dist[nb_id] = dist[cur_id] + weight;
          prev[nb_id] = cur_id;
          has_updated = true;
        }
      }
    }
    if(!has_updated)
      break;
  }
  
  // transform prev to path
  std::vector<std::string> path;
  std::string cur_id = end_id;
  while(cur_id != start_id) {
    path.push_back(cur_id);
    if(prev.find(cur_id) == prev.end())
      return {};
    cur_id = prev[cur_id];
  }

  path.push_back(start_id);

  // reverse
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * Traveling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path, 
 *                                                                      for example: {10.3, {{0, 1, 2, 3, 4, 0}, {0, 1, 2, 3, 4, 0}, {0, 4, 3, 2, 1, 0}}},
 *                                                                      where 10.3 is the total distance, 
 *                                                                      and the first vector is the path from 0 and travse all the nodes and back to 0,
 *                                                                      and the second vector is the path shorter than the first one,
 *                                                                      and the last vector is the shortest path.
 */
// Please use brute force to implement this function, ie. find all the permutations.
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  // initialize current path
  std::vector<std::string> cur;
  cur.push_back(location_ids[0]);
  std::unordered_map<std::string, bool> visited;
  visited[location_ids[0]] = true;

  // set min_dist to max
  records.first = DBL_MAX;

  // brute force
  all_permutation(location_ids, visited, cur, records);


  return records;
}

void TrojanMap::all_permutation(std::vector<std::string> &location_ids, std::unordered_map<std::string, bool> &visited, std::vector<std::string> &cur, std::pair<double, std::vector<std::vector<std::string>>> &records) {
  if(cur.size() == location_ids.size()) {
    // push starting location id into path
    cur.push_back(location_ids[0]);

    // compute path distance
    double cur_dist = CalculatePathLength(cur);

    // update path and distance
    if(cur_dist <= records.first) {
      records.first = std::min(records.first, cur_dist);
      records.second.push_back(cur);
    }
    

    // pop
    cur.pop_back();
    return;
  }

  // keep updating the path
  for(std::string &location_id : location_ids) {
    if(!visited[location_id]) {
      // mark visited
      visited[location_id] = true;

      // update current path
      cur.push_back(location_id);

      // DFS
      all_permutation(location_ids, visited, cur, records);

      // pop
      cur.pop_back();

      // mark unvisited
      visited[location_id] = false;
    }
  }
}

// Please use backtracking to implement this function
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;

  // initialize current path
  std::vector<std::string> cur;
  cur.push_back(location_ids[0]);
  std::unordered_map<std::string, bool> visited;
  visited[location_ids[0]] = true;

  // set min_dist to max
  records.first = DBL_MAX;

  // brute force
  backtracking_helper(location_ids, visited, cur, records);
  
  return records;
}

void TrojanMap::backtracking_helper(std::vector<std::string> &location_ids, std::unordered_map<std::string, bool> &visited, std::vector<std::string> &cur, std::pair<double, std::vector<std::vector<std::string>>> &records) {
  if(cur.size() == location_ids.size()) {
    // push starting location id into path
    cur.push_back(location_ids[0]);

    // compute path distance
    double cur_dist = CalculatePathLength(cur);

    // update path and distance
    if(cur_dist <= records.first) {
      records.first = std::min(records.first, cur_dist);
      records.second.push_back(cur);
    }
    

    // pop
    cur.pop_back();
    return;
  }

  // keep updating the path
  for(std::string &location_id : location_ids) {
    if(!visited[location_id]) {
      // mark visited
      visited[location_id] = true;

      // update current path
      cur.push_back(location_id);

      // DFS
      // backtracking technique
      double cur_dist = CalculatePathLength(cur);
      if(cur_dist < records.first)
        backtracking_helper(location_ids, visited, cur, records);

      // pop
      cur.pop_back();

      // mark unvisited
      visited[location_id] = false;
    }
  }
}

// Hint: https://en.wikipedia.org/wiki/2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;

  // initialize the current path
  std::vector<std::string> cur_path = location_ids;
  cur_path.push_back(location_ids[0]);

  // compute current best distance, and initialize records
  double best_dist = CalculatePathLength(cur_path);
  records.first = best_dist;
  records.second.push_back(cur_path);

  // 2-opt
  bool improved = true;

  while(improved) {
    improved = false;

    // iterate through all edges
    for(int i = 0; i < location_ids.size(); i++) {
      for(int j = i + 1; j < location_ids.size(); j++) {
        // do not swap adjacent nodes
        if (j == i + 1) continue;

        // generate new path
        std::vector<std::string> new_path = cur_path;

        // reverse order of i to j
        std::reverse(new_path.begin() + i + 1, new_path.begin() + j + 1);
        // new_path.back() = new_path.front();

        // compute new distance
        double new_dist = CalculatePathLength(new_path);

        // update if new_dist is shorter
        if(new_dist < best_dist) {
          improved = true;
          best_dist = new_dist;
          cur_path = new_path;
          records.first = new_dist;
          records.second.push_back(new_path);
        }
      }
    }
  }

  return records;
}

// This is optional
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_3opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_locations.csv"
 *   File content:
 *    Name
 *    Ralphs
 *    KFC
 *    Chick-fil-A
 *   Output: ['Ralphs', 'KFC', 'Chick-fil-A']
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(
    std::string locations_filename) {
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, word)) {
    location_names_from_csv.push_back(word);
  }
  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example: 
 *   Input: "topologicalsort_dependencies.csv"
 *   File content:
 *     Source,Destination
 *     Ralphs,Chick-fil-A
 *     Ralphs,KFC
 *     Chick-fil-A,KFC
 *   Output: [['Ralphs', 'Chick-fil-A'], ['Ralphs', 'KFC'], ['Chick-fil-A', 'KFC']]
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(
    std::string dependencies_filename) {
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    std::vector<std::string> dependency;
    while (getline(s, word, ',')) {
      dependency.push_back(word);
    }
    dependencies_from_csv.push_back(dependency);
  }
  fin.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a
 * sorting of nodes that satisfies the given dependencies. If there is no way to
 * do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(
    std::vector<std::string> &locations,
    std::vector<std::vector<std::string>> &dependencies) {
  // create adjacency list and in-table for each node
  std::unordered_map<std::string, std::vector<std::string>> adj_list;
  std::unordered_map<std::string, int> in_table;

  for(auto &name : locations) {
    in_table[name] = 0;
    adj_list[name];
  }

  // initialize adj_list and in_table
  for(auto &dep : dependencies) {
    adj_list[dep[0]].push_back(dep[1]);
    in_table[dep[1]]++;
  }

  // std::cout << adj_list.size() << std::endl;
  // std::cout << in_table.size() << std::endl;

  // the smallest value in in_table is the starting node of the graph
  std::vector<std::string> roots;
  for(auto &p : in_table) {
    if(p.second == 0)
      roots.push_back(p.first);
  }

  // print out roots
  for(const auto &name : roots)
    std::cout << name << std::endl;
  
  // std::cout << locations.size() << std::endl;
  // std::cout << "dependency size: " << dependencies.size() << std::endl;
  std::vector<std::string> result;
  std::unordered_map<std::string, bool> visited;
  DFS_topological_sort(adj_list, visited, result, roots[0]);
  
  // reverse the result
  std::reverse(result.begin(), result.end());
  return result;     
}

void TrojanMap::DFS_topological_sort(std::unordered_map<std::string, std::vector<std::string>> &adj_list,
                                     std::unordered_map<std::string, bool> &visited, std::vector<std::string> &result,
                                     std::string current) {
  // if all the neighbors are visisted, push into the result
  for(auto &nb : adj_list[current]) {
    if(visited[nb])
      continue;
    visited[nb] = true;
    DFS_topological_sort(adj_list, visited, result, nb);
  }

  result.push_back(current);
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  double lon_left = square[0], lon_right = square[1], lat_up = square[2], lat_low = square[3];
  double cur_lon = GetLon(id);
  double cur_lat = GetLat(id);
  
  return (cur_lon >= lon_left && cur_lon <= lon_right && cur_lat >= lat_low && cur_lat <= lat_up);
}


/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location
 * ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square
 * area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the
 * square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  for(auto &p : data) {
    // check latitude and longtitude is in square
    std::string cur_id = p.first;
    if(inSquare(cur_id, square))
      subgraph.push_back(p.first);
  }

  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true
 * if there is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the
 * square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  // udsing DFS to do cycle detection
  for(const std::string &id : subgraph) {
    std::unordered_map<std::string, bool> visited;
    if(!visited[id] && DFS_cycle_detection_helper(id, "", square, visited))
      return true;
  }
  
  return false;
}

bool TrojanMap::DFS_cycle_detection_helper(const std::string &cur_id, const std::string &prev_id, std::vector<double> &square, std::unordered_map<std::string, bool> &visited) {
  visited[cur_id] = true;
  for(const auto &nb_id : data[cur_id].neighbors) {
    // neighbor is previous node, skip this case
    if(nb_id == prev_id)
      continue;
    
    // neighbor might be outside of the square, skip these cases
    if(!inSquare(nb_id, square))
      continue;
    
    // found circle, since the neighbor is neither parent node nor outside of the square
    if(visited[nb_id])
      return true;
    
    // not found yet
    if(DFS_cycle_detection_helper(nb_id, cur_id, square, visited))
      return true;
  }
    
  
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and
 * return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {double} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  std::unordered_map<std::string, bool> visited;


  // use min_heap
  auto cmp = [](const std::pair<double, std::string>& a, const std::pair<double, std::string>& b) {
    return a.first > b.first;
  };
  std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, decltype(cmp)> pq(cmp);
  // std::priority_queue<std::pair<double, std::string>> pq;

  // get current id
  std::string center_id = GetID(name);
  visited[center_id] = true;

  // push neighbors of center into queue
  for(auto nb_id : data[center_id].neighbors) {
    double nb_dist = CalculateDistance(center_id, nb_id);
    pq.push({nb_dist, nb_id});
  }

  // BFS
  while(!pq.empty()) {
    // compute distance of top node's neighbors
    double cur_dist = pq.top().first;
    std::string cur_id = pq.top().second;
    pq.pop();

    // check current node matches the attribute
    if(data[cur_id].attributes.find(attributesName) != data[cur_id].attributes.end() && cur_dist <= r) {
      res.push_back(cur_id);
    }

    // check all neighbors
    for(auto nb_id : data[cur_id].neighbors) {
      if(!visited[nb_id]) {
        // mark visited and calculate distance
        visited[nb_id] = true;
        double nb_dist = CalculateDistance(center_id, nb_id);

        // if neighbor is in the range, push into priority queue
        if(nb_dist <= r)
          pq.push({nb_dist, nb_id});
      }
    }
  }

  return res;
}

/**
 * Shortest Path to Visit All Nodes: Given a list of locations, return the shortest
 * path which visit all the places and no need to go back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::vector<std::string> }      : the shortest path
 */
std::vector<std::string> TrojanMap::TrojanPath(
      std::vector<std::string> &location_names) {
    std::vector<std::string> res;
    
    // using 2-opt approach to find the order of locations, and slightly modified the code
    std::pair<double, std::vector<std::vector<std::string>>> records;

    // initialize the current path, path is represented in ids
    std::vector<std::string> cur_path;
    for(std::string location_name : location_names)
        cur_path.push_back(GetID(location_name));

    // compute current best distance, and initialize records
    double best_dist = CalculatePathLength(cur_path);
    records.first = best_dist;
    records.second.push_back(cur_path);

    // 2-opt
    bool improved = true;

    while(improved) {
      improved = false;

      // iterate through all edges
      for(int i = 0; i < cur_path.size() - 1; i++) {
        for(int j = i + 1; j < cur_path.size(); j++) {
          // generate new path
          std::vector<std::string> new_path = cur_path;

          // reverse order of i to j
          std::reverse(new_path.begin() + i, new_path.begin() + j + 1);
          // new_path.back() = new_path.front();

          // compute new distance
          double new_dist = CalculatePathLength(new_path);

          // update if new_dist is shorter
          if(new_dist < best_dist) {
            improved = true;
            best_dist = new_dist;
            cur_path = new_path;
            records.first = new_dist;
            records.second.push_back(new_path);
          }
        }
      }
    }

    // use Dijkstra to store the path
    for(int i = 1; i < records.second.back().size(); i++) {
      std::string start_name = GetName(records.second.back()[i - 1]);
      std::string end_name = GetName(records.second.back()[i]);
      std::vector<std::string> path = CalculateShortestPath_Dijkstra(start_name, end_name);
      if(i == 1) {
        res = path;
      } else {
        for(int j = 1; j < path.size(); j++) {
          res.push_back(path[j]);
        }
      }
    }
    // res = records.second.back();

    return res;
}

/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries 
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>>& q) {
    std::vector<bool> ans;


    for(auto &p : q) {
      std::queue<std::string> qq;
      std::unordered_set<std::string> visited;
      double tank = p.first;
      std::string start_id = GetID(p.second[0]);
      std::string end_id = GetID(p.second[1]);

      // check if ids are valid
      if(start_id == "" || end_id == "") {
        ans.push_back(false);
        continue;
      }

      qq.push(start_id);
      visited.insert(start_id);

      // std::cout << start_id << std::endl;
      // std::cout << end_id << std::endl;

      // BFS
      bool enough_fuel = false;
      while(!qq.empty()) {
        std::string cur_id = qq.front();
        qq.pop();

        if(cur_id == end_id) {
          // std::cout << "found route..." << std::endl;
          enough_fuel = true;
          break;
        }
        
        // check all neighbors
        for(std::string nb_id : data[cur_id].neighbors) {
          //  check visited
          if(visited.find(nb_id) == visited.end()) {
            // check distance
            double dist = CalculateDistance(cur_id, nb_id);
            // std::cout << GetName(cur_id) << " to " << GetName(nb_id) << ": " << dist << std::endl;
            if(dist <= tank) {
              visited.insert(nb_id);
              qq.push(nb_id);
            }
          } 
        }
      }
      // std::cout << enough_fuel << std::endl;
      ans.push_back(enough_fuel);
    }
    
    return ans;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * We have provided the code for you. Please do not need to change this function.
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0])) n.attributes.insert(word);
        if (isdigit(word[0])) n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
