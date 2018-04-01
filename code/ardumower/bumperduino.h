void Robot::readBumpderuino() {
  // serial input
     static char cmd = 0;     
     static char cma = 0;
     static char cmb = 0;
     static String stringReceived = "";
     static boolean CmdComplete = false;
     static boolean CmaComplete = false;
  while (Serial1.available() > 0) {
     if (Serial1.available() > 0) {
      cmd = (char)Serial1.read();
      //if(cmd>'Z') cmd-=32;
      if (cmd == '\t') CmdComplete = true; 
      else if (((cmd == 'Y') || (cmd == 'P') || (cmd == 'R') || (cmd == 'A') || (cmd == 'G') || (cmd == 'M') || (cmd == 'H') 
              || (cmd == 'B') || (cmd == 'S') || (cmd == 'W') || (cmd == 'D') || (cmd == 'T')) && (CmaComplete == false)){
        stringReceived = "";
        cma = cmd;
        CmaComplete = true;
      }
      else if (((cmd == 'L') || (cmd == 'R') || (cmd == 'C') || (cmd == 'X') 
              || (cmd == 'Y') || (cmd == 'Z') || (cmd == 'T')) && CmaComplete)cmb = cmd;
      else if (cmd == 10) break; 
      else  stringReceived += cmd;
      
      if (CmdComplete){
        /*
       Debug.print(cma);
       Debug.print(cmb);
       Debug.println(stringReceived);
       */
       switch(cma) {
        case 'Y':if (isValidNumber(stringReceived)) imu.ypr.yaw = stringReceived.toFloat();stringReceived = "";  break;
        case 'P': if (isValidNumber(stringReceived)) imu.ypr.pitch = stringReceived.toFloat()/180.0f*PI;stringReceived = ""; break;             
        case 'R': if (isValidNumber(stringReceived)) imu.ypr.roll = stringReceived.toFloat()/180.0f*PI; stringReceived = ""; break;
        case 'D': if (isValidNumber(stringReceived)) bumperduinoDt = stringReceived.toFloat(); stringReceived = ""; break;
        case 'T': if (isValidNumber(stringReceived)) imuTemperature = stringReceived.toFloat(); stringReceived = ""; break;
        
        //case 'H': if (isValidNumber(stringReceived)) imu.ypr.yaw = stringReceived.toFloat(); stringReceived = ""; break;
        case 'B': switch(cmb) {
                    case 'L': if (isValidNumber(stringReceived)) bumperLeftNew = stringReceived.toInt(); stringReceived = ""; break;
                    case 'R': if (isValidNumber(stringReceived)) bumperRightNew = stringReceived.toInt();stringReceived = ""; break;
                    default: break;
                    }
                    break;         
        case 'S': switch(cmb) {
                    case 'L': if (isValidNumber(stringReceived)) sonarDistLeftNew = stringReceived.toFloat(); stringReceived = ""; break;
                    case 'R': if (isValidNumber(stringReceived)) sonarDistRightNew = stringReceived.toFloat();stringReceived = ""; break;
                    case 'C': if (isValidNumber(stringReceived)) sonarDistCenterNew = stringReceived.toFloat();stringReceived = ""; break;
                    default: stringReceived = ""; break;
                    }
                    break;  
       case 'A': switch(cmb) {
                    case 'X': if (isValidNumber(stringReceived)) imu.acc.x = stringReceived.toFloat()*10.0f; stringReceived = ""; break;
                    case 'Y': if (isValidNumber(stringReceived)) imu.acc.y = stringReceived.toFloat()*10.0f; stringReceived = ""; break;
                    case 'Z': if (isValidNumber(stringReceived)) imu.acc.z = stringReceived.toFloat()*10.0f; stringReceived = ""; break;
                    default: stringReceived = ""; break;
                    }
                    break;
        case 'G': switch(cmb) {
                    case 'X': if (isValidNumber(stringReceived)) imu.gyro.x = stringReceived.toFloat(); stringReceived = ""; break;
                    case 'Y': if (isValidNumber(stringReceived)) imu.gyro.y = stringReceived.toFloat();stringReceived = ""; break;
                    case 'Z': if (isValidNumber(stringReceived)) imu.gyro.z = stringReceived.toFloat();stringReceived = ""; break;
                    default: stringReceived = ""; break;
                    }
                    break;  
        case 'M': switch(cmb) {
                    case 'X': if (isValidNumber(stringReceived)) imu.com.x = stringReceived.toFloat(); stringReceived = ""; break;
                    case 'Y': if (isValidNumber(stringReceived)) imu.com.y = stringReceived.toFloat();stringReceived = ""; break;
                    case 'Z': if (isValidNumber(stringReceived)) imu.com.z = stringReceived.toFloat();stringReceived = ""; break;
                    default: stringReceived = ""; break;
                    }
                    break;          
        case 'W': if ( (stateCurr != STATE_OFF) && (stateCurr != STATE_MANUAL) && (stateCurr != STATE_STATION) 
                    && (stateCurr != STATE_STATION_CHARGING) && (stateCurr != STATE_STATION_CHECK) 
                    && (stateCurr != STATE_STATION_REV) && (stateCurr != STATE_STATION_ROLL) 
                    && (stateCurr != STATE_STATION_FORW) && (stateCurr != STATE_REMOTE) && (stateCurr != STATE_PERI_OUT_FORW)
                    && (stateCurr != STATE_PERI_OUT_REV) && (stateCurr != STATE_PERI_OUT_ROLL) && (stateCurr != STATE_PERI_TRACK)
                    && (stateCurr != STATE_IMU_CALIB)){
                      setNextState(STATE_IMU_CALIB, 0);
                    }
                    else {
                      imuCalibTime = currentMillis;
                      bumperduinoNotStarted = true;
                    }
                    break;
        case 10: stringReceived = "";return;
        case 9: stringReceived = "";break;
        default:
          /*while(cmd != 10){
            if (cmd >= 32){
              stringReceived += cmd;
            }
            cmd = (char)Serial1.read();
            
          }*/
          //Debug.println(stringReceived);
          stringReceived = "";
          break;
        }
        CmdComplete = false;
        CmaComplete = false;
      }
    }
    //else break;
  }
}
void Robot::initBumpderuino(){
  Serial1.begin(115200);
}

void Robot::imuStartBumpderuino(){
  Serial1.println("a");
}

boolean Robot::isValidNumber(String str){
   boolean isNum=false;
   if(!(str.charAt(0) == '+' || str.charAt(0) == '-' || isDigit(str.charAt(0)))) return false;

   for(byte i=1;i<str.length();i++)
   {
       if(!(isDigit(str.charAt(i)) || str.charAt(i) == '.')) return false;
   }
   return true;
}
