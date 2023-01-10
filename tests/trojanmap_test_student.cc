#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, FindPosition) {
  TrojanMap m;
  {
  EXPECT_EQ(m.GetLat("4540761794") , 34.0189842);
  EXPECT_EQ(m.GetLon("4540761794") , -118.2876448);
  EXPECT_EQ(m.GetName("4540761794") , "Nekter Juice Bar");
  EXPECT_EQ(m.GetID("Nekter Juice Bar") , "4540761794");
  std::vector<std::string> neighbours = {"4540690064"};
  EXPECT_EQ(m.GetNeighborIDs("4540761794") , neighbours);
  EXPECT_EQ(m.GetPosition("Nekter Juice Bar").first, 34.0189842);
  EXPECT_EQ(m.GetPosition("Nekter Juice Bar").second, -118.2876448);
  }

  {
  EXPECT_EQ(m.GetLat("00000000") , -1);
  EXPECT_EQ(m.GetLon("00000000") , -1);
  EXPECT_EQ(m.GetName("00000000") , "NULL");
  EXPECT_EQ(m.GetID("Nowhere") , "");
  std::vector<std::string> neighbours = {};
  EXPECT_EQ(m.GetNeighborIDs("00000000") , neighbours);
  EXPECT_EQ(m.GetPosition("Nowhere").first, -1);
  EXPECT_EQ(m.GetPosition("NOwhere").second, -1);
  }
}

// Following test case for Autocomplete is given in trojanmap_test.
// Because it's impossible for us to count thousands of entries of data.
TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Chi");
  std::unordered_set<std::string> gt = {"Chick-fil-A", "Chipotle", "Chinese Street Food"}; // groundtruth for "Ch"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("chi");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("cHi"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("CHI"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("Night", "it"), 3);
  EXPECT_EQ(m.CalculateEditDistance("washington", "washington"), 0);
  EXPECT_EQ(m.CalculateEditDistance("Angel", ""), 5);
}

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Halbox"), "Holbox");
  EXPECT_EQ(m.FindClosestName("Smoking Shop"), "Smoke Shop");
  EXPECT_EQ(m.FindClosestName("kfc"), "KFC");
}

TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;

  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375", "5559640911", "6787470571", "6808093910", "6808093913", "6808093919", 
      "6816831441", "6813405269", "6816193784", "6389467806", "6816193783", "123178876", 
      "2613117895", "122719259", "2613117861", "6817230316", "3642819026", "6817230310", 
      "7811699597", "5565967545", "123318572", "6813405206", "6813379482", "544672028", 
      "21306059", "6813379476", "6818390140", "63068610", "6818390143", "7434941012", 
      "4015423966", "5690152766", "6813379440", "6813379466", "21306060", "6813379469", 
      "6813379427", "123005255", "6807200376", "6807200380", "6813379451", "6813379463", 
      "123327639", "6813379460", "4141790922", "4015423963", "1286136447", "1286136422", 
      "4015423962", "6813379494", "63068643", "6813379496", "123241977", "4015372479", 
      "4015372477", "1732243576", "6813379548", "4015372476", "4015372474", "4015372468", 
      "4015372463", "6819179749", "1732243544", "6813405275", "348121996", "348121864", 
      "6813405280", "1472141024", "6813411590", "216155217", "6813411589", "1837212103", 
      "1837212101", "6820935911", "4547476733" };
  
  // Print the path lengthsC
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  //std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;

  for(long unsigned int i = 0; i < path.size(); i++){
    if( i%5 == 0){
      std::cout << std::endl;
    }
    std::cout << path[i] << ',';
  }
  std::cout << std::endl;
  //EXPECT_EQ(path, gt);
  
  
  // Reverse the input from Ralphs to Chick-fil-A
  auto path_2 = m.CalculateShortestPath_Dijkstra("Chick-fil-A", "Ralphs");
  //std::reverse(gt.begin(),gt.end()); // Reverse the path

  for(int i = 0; i < path_2.size(); i++){
    if( i%5 == 0){
      std::cout << std::endl;
    }
    std::cout << path_2[i] << ',';
  }
  

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  //std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  //EXPECT_EQ(path, gt);
  
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Target");
  std::vector<std::string> gt{
      "2578244375","5559640911","6787470571","6808093910","8410528464","8410528457","6808093913",
      "6808093919","6816831441","6813405269","6816193784","6389467806","6816193783","123178876",
      "2613117895","122719259","6807243574","6807243576","213332111","441895337","441895335",
      "122719255","2613117893","6813405231","6813405237","6813405235","6047197523","6813379584","5237417650"}; // Expected path
  
  for(long unsigned int i = 0; i < path.size(); i++){
    if( i%5 == 0){
      std::cout << std::endl;
    }
    std::cout << path[i] << ',';
  }

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  //std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  //EXPECT_EQ(path, gt);
  
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  //std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  //EXPECT_EQ(path, gt);
  
}

TEST(TrojanMapTest, TSP1) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  
  std::vector<std::string> output = result.second[result.second.size()-1];
  for(auto& entry : output){
    std::cout << entry << ", ";
  }
  std::cout << std::endl;
}

TEST(TrojanMapTest, TSP3_2) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976", "6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP3_3) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976", "1873055993"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976", "1873055993", "6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}


TEST(TrojanMapTest, TSP_OPT3) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  
  std::vector<std::string> output = result.second[result.second.size()-1];
  for(auto& entry : output){
    std::cout << entry << ", ";
  }
  std::cout << std::endl;
}

TEST(TrojanMapTest, TSP_OPT3_2) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_OPT3_3) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976", "1873055993"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976", "1873055993", "6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

// Test cycle detection function
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  // for(long unsigned int i = 0; i < sub1.size(); i++){
  //   if( i%5 == 0){
  //     std::cout << std::endl;
  //   }
  //   std::cout <<sub1[i] << ',';
  // }
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.289, 34.030, 34.020};
  auto sub2 = m.GetSubgraph(square2);
  // for(long unsigned int i = 0; i < sub2.size(); i++){
  //   if( i%5 == 0){
  //     std::cout << std::endl;
  //   }
  //   std::cout <<sub2[i] << ',';
  // }
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);
}


// Test topology function
TEST(TrojanMapTest, TopologicalSort) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"KFC","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs", "KFC","Chick-fil-A"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, TopologicalSort_2) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"A", "B", "C", "D", "E"};
  std::vector<std::vector<std::string>> dependencies = {{"A","B"}, {"A","C"}, {"A","E"}, {"B","C"}, {"B","D"}, {"C","D"}, {"C","E"},{"D","E"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"A","B","C","D","E"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, TopologicalSort_3) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"A", "B", "C", "D", "E"};
  std::vector<std::vector<std::string>> dependencies = {{"A","B"}, {"A","C"}, {"A","E"}, {"B","C"}, {"B","D"}, {"C","D"}, {"C","E"},{"D","E"},{"C","A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, TopologicalSort_4) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"A", "B", "C", "D", "E"};
  std::vector<std::vector<std::string>> dependencies = {{"A","B"}, {"B","C"}, {"C","D"}, {"D","E"},{"E","C"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapTest, ReadFromFile) {
  TrojanMap m;
  std::vector<std::string> locations = m.ReadLocationsFromCSVFile("/home/yuqi/final-project-Yuqi-C/input/topologicalsort_locations.csv");
  std::vector<std::vector<std::string>> dependencies = m.ReadDependenciesFromCSVFile("/home/yuqi/final-project-Yuqi-C/input/topologicalsort_dependencies.csv");
  for(auto name : locations){
    std::cout << name << std::endl;
  }
  std::cout << "******************" << std::endl;
  for(auto depen : dependencies){
    std::cout << depen[0] << " " << depen[1] << std::endl;
  }
  std::cout << "******************" << std::endl;
}

// Test FindNearby points
TEST(TrojanMapTest, FindNearby) {
  TrojanMap m;
  
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 10);
  std::vector<std::string> ans{"5237417649", "6045067406", "7158034317"};
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby_2) {
  TrojanMap m;
  
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 0);
  std::vector<std::string> ans;
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby_3) {
  TrojanMap m;
  
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 1);
  std::vector<std::string> ans{"5237417649"};
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby_4) {
  TrojanMap m;
  
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 2);
  std::vector<std::string> ans{"5237417649", "6045067406"};
  EXPECT_EQ(result, ans);
}

TEST(TrojanMapTest, FindNearby_5) {
  TrojanMap m;
  
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 3);
  std::vector<std::string> ans{"5237417649", "6045067406", "7158034317"};
  EXPECT_EQ(result, ans);
}
