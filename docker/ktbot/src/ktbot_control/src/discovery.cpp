#include <string.h>
#include <regex>
#include <stdio.h>
#include <iostream>

using std::string;
using std::map;
using std::vector;

void push_resolved_files(vector<string>& fileset, const string& glob) {
  FILE *in = popen("ls " + glob, "r");
  while (fgets(line, line_size, in)) {
     fileset.push(line);
  }
}

string get_path_for_serial_id(string uid) {  
  char line[line_size];
  string df;
  vector<string> paths;
  push_resolved_files(paths, "/dev/ttyUSB*");
  push_resolved_files(paths, "/dev/ttyACM*");
  
  std::regex match_serial_uid("ID_SERIAL=(.+?)\n");

  for (const string& path : paths) {
    if (path == "") {
      continue;
    }
    FILE *u = popen("udevadm info -a --query=property -n " + path, "r");
    string output;
    while (fgets(line, line_size, u)) {
      output += line;
    }

    std::cmatch cm;    // same as std::match_results<const char*> cm;
    std::regex_match(output.c_str(),cm,match_serial_uid);
    if (cm.size() == 2 && cm[1] == uid) {
      return path;
    }
  }
  return "";
}