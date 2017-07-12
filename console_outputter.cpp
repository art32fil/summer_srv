#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <string>
#include <sstream>
#include "labirint_msg/message.h"
#include "labirint_srv/service.h"


#include <iostream>

using namespace std;

class Field {
  bool** data;
  int x;
  int y;
  int size_x;
  int size_y;

  bool position_is_possible(int x, int y) const {
    return data[x][y];
  }

public:
  Field(int size_x = 10, int size_y = 10, int pos_x = 5, int pos_y = 5) {
    x = pos_x; y = pos_y;
    this->size_x = size_x; this->size_y = size_y;
    data = new bool*[size_x];
    for (int x = 0; x < size_x; x++) {
      data[x] = new bool[size_y];
      for (int y = 0; y < size_y; y++) {
        data[x][y] = true;
      }
    }
  }

  friend ostream& operator << (ostream& ostr, const Field& f) {
    system("clear");
    ostr << "┌"; for (int i = 0; i < f.size_x; i++) { ostr << "―"; } ostr << "┐" << endl;
    for (int y = 0; y < f.size_y; y++) {
      ostr << "|";
      for (int x = 0; x < f.size_x; x++) {
        if (x == f.x && y == f.y)
          ostr << "x";
        else if (f.data[x][y] == true)
          ostr << " ";
        else if (f.data[x][y] == false)
          ostr << "8";
        else
          ostr << "?";
      }
      ostr << "|" << endl;
    }
    ostr << "└"; for (int i = 0; i < f.size_x; i++) { ostr << "―"; } ostr << "┘" << endl;
    return ostr;
  }

  void move_up ()    { if (y > 0        && position_is_possible(x, y-1)) y--; }
  void move_down ()  { if (y < size_y-1 && position_is_possible(x, y+1)) y++; }
  void move_left ()  { if (x > 0        && position_is_possible(x-1, y)) x--; }
  void move_right () { if (x < size_x-1 && position_is_possible(x+1, y)) x++; }

  void load(ifstream& fin) {
    for (int x = 0; x < size_x; x++) {
      delete[] data[x];
    }
    delete[] data;

    string str_size_x, str_size_y;
    getline(fin, str_size_x);
    getline(fin, str_size_y);
    stringstream sstr_x, sstr_y;
    sstr_x << str_size_x.c_str();
    sstr_x >> size_x;
    sstr_y << str_size_y.c_str();
    sstr_y >> size_y;

    data = new bool*[size_x];
    for (int x = 0; x < size_x; x++) {
      data[x] = new bool[size_y];
    }
    for (int y = 0; y < size_y; y++) {
      string line;
      getline(fin,line);
      for (int x = 0; x < size_x; x++) {
        if (line[x] == ' ')
          data[x][y] = true;
        else
          data[x][y] = false;
      }
    }
  }

} field;

void handle_function(const labirint_msg::message& msg) {
  switch (msg.command) {
  case 'w':
    if (msg.t.sec % 2)
      field.move_up();
    else
      field.move_down();
    break;
  case 's':
    if (msg.t.sec % 2)
      field.move_down();
    else
      field.move_up();
    break;
  case 'a':
    if (msg.t.sec % 2)
      field.move_left();
    else
      field.move_right();
    break;
  case 'd':
    if (msg.t.sec % 2)
      field.move_right();
    else
      field.move_left();
    break;
  default :
    break;
  }
  cout << field << endl;
}

bool load_file(labirint_srv::service::Request& req,
               labirint_srv::service::Response& res) {
  ifstream fin(req.file.c_str());
  res.result = fin.is_open() && !fin.eof();
  field.load(fin);
  cout << field << endl;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"command_subscriber");
  ros::NodeHandle n;
  ros::Subscriber subscriber = n.subscribe("command_topic", 1000, handle_function);
  ros::ServiceServer service = n.advertiseService("upload_new_world", load_file);
  cout << field << endl;
  ros::spin();
  return 0;
}

