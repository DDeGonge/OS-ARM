// Read serial messages if exist
bool respondToSerial(char (&serial_data) [MAX_MSG_LEN])
{
  uint8_t index = 0;
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      char newchar = Serial.read();
      if ((newchar != '\n') and (index < MAX_MSG_LEN)) {
        serial_data[index] = newchar;
        index++;
      }
      else {
        break;
      }
    }
    return true;
  }
  return false;
}

void clear_data(char (&serial_data) [MAX_MSG_LEN]) {
  for (uint16_t i = 0; i < MAX_MSG_LEN; i++) {
    serial_data[i] = '\0';
  }
}

void parse_inputs(char serial_data[MAX_MSG_LEN], vector<string> &args) {
  char delim = ' ';
  uint32_t index = 0;
  string temp_arg_str = "";
//  debug_print_str(serial_data);

  while (serial_data[index] != '\0') {
    temp_arg_str += serial_data[index];
    index++;
    if (serial_data[index] == delim) {
      args.push_back(temp_arg_str);
      temp_arg_str = "";
      index++;
    }

    // timeout
    if (index > MAX_MSG_LEN) return;
  }
  args.push_back(temp_arg_str);
}

void debug_print_str(string str)
{
  for (uint16_t i = 0; i < str.length(); i++)
  {
    Serial.print(str[i]);
  }
  Serial.println();
}

void parse_int(string inpt, char &cmd, int32_t &value)
{
  cmd = '\0';
  value = NOVALUE;
  cmd = inpt[0];
  string temp_arg_char = "";
  for (uint32_t i = 1; i < inpt.length(); i++)
  {
    temp_arg_char += inpt[i];
  }

  value = stoi(temp_arg_char);
}


/* GCODE PARSER STUFF */

gcode_command_floats::gcode_command_floats(vector<string> inputs)
{
  if (inputs.size() == 1)
    return;

  for(uint16_t arg_i = 1; arg_i < inputs.size(); arg_i++)
  {
    char char_value = '\0';
    float float_value = NOVALUE;
    parse_float(inputs[arg_i], char_value, float_value);

    commands.push_back(tolower(char_value));
    values.push_back(float_value);
  }
}

float gcode_command_floats::fetch(char com_key)
{
  vector<char>::iterator itr = find(commands.begin(), commands.end(), com_key);
  if (itr != commands.cend())
  {
    return values[distance(commands.begin(), itr)];
  }

  return NOVALUE;
}

bool gcode_command_floats::com_exists(char com_key)
{
  vector<char>::iterator itr = find(commands.begin(), commands.end(), com_key);
  if (itr != commands.cend())
  {
    return true;
  }

  return false;
}

void gcode_command_floats::parse_float(string inpt, char &cmd, float &value)
{
  if (inpt.length() > 0)
  {
    cmd = inpt[0];
    if (inpt.length() == 1)
      return;

    string temp_arg_char = "";
    for (uint32_t i = 1; i < inpt.length(); i++)
    {
      temp_arg_char += inpt[i];
    }
  
    value = stof(temp_arg_char);
  }
}
