
bool receiving_ = false;
String data_buffer_ = "";
uint64_t ser_ref_time_;
const int ser_timeout_ = 1000; // microseconds
uint64_t ser_read_start_time_; 
int msg_element_len_ = 0; 
int msg_byte_len_ = 0; 
bool ser_first_call_ = true; 


enum class SerialByteType : char
{
  START_BYTE = '<',
  END_BYTE = '>',
  SPLIT_BYTE = ':',
  NEXT_JOINT = 'n',
  LAST_JOINT = 'l'
};


// for telling program what do do with values received over serial
enum class SerialMessageType 
{
  JOINT_TRAJECTORY_5D = 0
};

SerialMessageType msg_type_;


// for storing delay information
struct AngleArray5D
{
  static const int n_joints = 5;
  static const int MAX_LEN = 10000;
  bool ready_to_move = false; 
  int joint_idx = 0;
  int angle_idx[n_joints] = {0, 0, 0, 0, 0};
  float angles[n_joints][MAX_LEN];

  void reset()
  {
    ready_to_move = false;
    joint_idx = 0;

    for (int i = 0; i < n_joints; i++)
    {
      angle_idx[i] = 0; 
      for (int j = 0; j < MAX_LEN; j++)
      {
        angles[i][j] = -1; 
      }
    }
  }

  void printValues()
  {
    if (!ready_to_move)
    {
      return;
    }
    for (int i = 0; i < n_joints; i++)
    {
      Serial.print("J" + String(i) + ": [");
      for (int j = 0; j < MAX_LEN; j++)
      {
        if (j >= angle_idx[i] - 1)
        {
          Serial.print(String(angles[i][j]) + "]\n");
          break;
        }
        Serial.print(String(angles[i][j]) + ", ");
      }
    }
    ready_to_move = false;
  }
};

AngleArray5D angle_array_; 


void setup() 
{

  const int baudrate = 921600;
  Serial.begin(baudrate);
  Serial.println("Serial init");

  pinMode(LED_BUILTIN, OUTPUT);

  angle_array_.reset(); 

}



int readSerialBuffer(bool allow_timeout = false)
{
  ser_ref_time_ = micros(); 

  while (Serial.available() > 0)
  {
    // if this function has gone on for too long we need to 
    // get back to doing other stuff for a little while
    if (allow_timeout && micros() - ser_ref_time_ > ser_timeout_) return -1;
    
    char c = (char)Serial.read(); 
    SerialByteType cc = static_cast<SerialByteType>(c);

    if (cc == SerialByteType::START_BYTE) // begin new message
    {
      receiving_ = true; 
      data_buffer_ = ""; // reset buffer
      msg_byte_len_ = 0;
      msg_element_len_ = 0; 
      ser_first_call_ = true; 
      ser_read_start_time_ = micros(); 
    }
    else if (cc == SerialByteType::SPLIT_BYTE && receiving_) 
    {
      if (ser_first_call_) // if this is the first full value we have ingested
      {
        int ser_first_val = data_buffer_.toInt();
        msg_type_ = static_cast<SerialMessageType>(ser_first_val); // set message type 
        ser_first_call_ = false;
      }
      else 
      {
        parseDataBuffer();
      }
      data_buffer_ = ""; // reset buffer for next val
      msg_byte_len_++; 
      msg_element_len_++; 
    }
    else if (cc == SerialByteType::END_BYTE && receiving_) // end of message
    {
      parseDataBuffer();
      data_buffer_ = "";
      receiving_ = false;
      msg_byte_len_++; 
      msg_element_len_++;  
      
      auto elapsed = micros() - ser_read_start_time_; 
      Serial.println("Read took " + String(elapsed) + " microseconds");
      Serial.println("Read " + String(msg_byte_len_) + " bytes");
      Serial.println("Read " + String(msg_element_len_) + " elements");
    }
    else if (receiving_) // regular receieve, append to floatBuffer
    {
      data_buffer_ += c; 
      msg_byte_len_++; // only inc byte counter here
    }
  }

  return 0;
}



const int pulse_delay = 1000; // milliseconds
const float duty_cycle = 0.1; 
uint64_t pulse_ref_time; 

void pulseLED() // async pulse LED to signal MCU is alive
{
  auto elapsed = millis() - pulse_ref_time;
  if (elapsed < pulse_delay * duty_cycle)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (elapsed >= pulse_delay * duty_cycle && elapsed < pulse_delay)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    pulse_ref_time = millis(); 
  }
}


void handleJointTrajectory() // for parsing msg containing a set of joint angles
{
  /*
  AngleArray5D data example:
  <0:X:n:X:n:X:n:X:n:X:l>
  Explanation: joint angle value, followed by 'n' to signal next joint
  Last joint angle is followed by 'l' to signal end of message 
  */
  if (data_buffer_.length() == 1 && static_cast<SerialByteType>(data_buffer_[0]) == SerialByteType::NEXT_JOINT)
  {
    angle_array_.joint_idx++; 
    return;
  }
  else if (data_buffer_.length() == 1 && static_cast<SerialByteType>(data_buffer_[0]) == SerialByteType::LAST_JOINT)
  {
    angle_array_.ready_to_move = true; 
    return;
  }

  int i = angle_array_.joint_idx;
  int j = angle_array_.angle_idx[i];
  if (i > angle_array_.n_joints - 1 || j > angle_array_.MAX_LEN - 1) // we are outside bounds for some reason, BAD
  {
    Serial.println("Outside bounds - error");
    return;
  }
  angle_array_.angles[i][j] = data_buffer_.toFloat(); // assign value to pose array
  angle_array_.angle_idx[i]++; // inc so next value gets assigned to next position
}


void parseDataBuffer()
{
  switch(msg_type_)
  {
    case SerialMessageType::JOINT_TRAJECTORY_5D:
    {
      handleJointTrajectory();
    }
    default:
    {
      Serial.println("Unknown state");
      break;
    }
  }
}



// MAIN LOOP
void loop() 
{
  int ret = readSerialBuffer(false);
  angle_array_.printValues();
  pulseLED(); 
}
