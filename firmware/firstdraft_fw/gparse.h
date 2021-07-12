#include <string>
#include <vector>
#include <algorithm>

#define MAX_MSG_LEN 100
#define NOVALUE 999999

using namespace std;

/* SERIAL FUNCS */
void clear_data(char (&serial_data) [MAX_MSG_LEN]);
bool respondToSerial(char (&serial_data) [MAX_MSG_LEN]);

/* PARSING FUNCS */
void parse_inputs(char serial_data[MAX_MSG_LEN], vector<string> &args);
void parse_int(string inpt, char &cmd, int32_t &value);

/* DEBUG FUNCS */
void debug_print_str(string str);

struct gcode_command_floats
{
  gcode_command_floats(vector<string> inputs);

  public:
  float fetch(char com_key);
  bool com_exists(char com_key);
  

  private:
  void parse_float(string inpt, char &cmd, float &value);

  vector<char> commands;
  vector<float> values;
};
