/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238;

import java.io.File;
import java.io.FileWriter;
import java.util.HashMap;

public final class Logger {

  private static final String newline = "\n";
  //static Boolean isDebug;
  static Boolean outputToLogFile;
  static Level logLevel = Level.Debug;

  /**
   * Checks on the SmartDashboard on if debug is true
   * 
   * @return
   */
  public static Boolean isDebug() {

    // isDebug = SmartDashboard.getBoolean("Debug", false);

    // return isDebug;
    return true;
  }

  /**
   * Checks on the SmartDashboard on if "Output Log to File" is true
   * 
   * @return
   */
  public static Boolean writeToFile() {

    // outputToLog = SmartDashboard.getBoolean("Output Log to File", isDebug);

    // return outputToLog;
    return false;
  }

  // getCurrentDate()??

  // clearLog()??

  /**
   * Logs a string to the system log Writes the comment to logFile238 by default
   * Also writes it to a file if writeToFile is true on the SmartDashboard
   * 
   * @param comment
   */
  public static void Debug(String comment) {
    log(Level.Debug, comment);
  }

  public static void Trace(String comment) {
    log(Level.Trace, comment);
  }

  public static void Info(String comment) {
    log(Level.Info, comment);
  }

  public static void Warn(String comment) {
    log(Level.Warn, comment);
  }

  public static void Error(String comment) {
    log(Level.Error, comment);
  }

  private static void log(Level level, String message){
    if (level.value >= logLevel.value){
      System.out.println(message);

      if (writeToFile()){
        writeToLogFile(message);
      }
    }
  }

  
    /**
   * Logs a string to the system log and creates a new file for it
   * Can be used for categorization
   * Also writes it to a file if writeToFile is true on the SmartDashboard
   * @param comment
   */
  // private static void Log(String comment,String fileName)
  // {
  //   if(isDebug())
  //   {
  //     //System.out.println(comment);
      
  //     if( writeToFile() ){
        
  //       writeToNewLogFile(comment,fileName);
        
  //     }
      
  //   }
    
  // }

  // /**
  //  * Writes the log into a custom file
  //  * 
  //  * @param log
  //  */
  // private static void writeToNewLogFile(String log, String logFileName) {

  //   try {

  //     File customFile = new File("/home/lvuser/" + logFileName + ".txt");

  //     // If the file already exists, open the file and write the string to it
  //     if (customFile.exists()) {

  //       FileWriter logFile = new FileWriter("/home/lvuser/" + logFileName + ".txt", true);
  //       logFile.write(newline + log);
  //       logFile.flush();
  //       logFile.close();

  //       // If the file doesn't already exists, create a new one and write the string to
  //       // it
  //     } else {

  //       customFile.createNewFile();
  //       FileWriter logFile = new FileWriter("/home/lvuser/" + logFileName + ".txt", true);
  //       logFile.write(newline + log);
  //       logFile.flush();
  //       logFile.close();

  //     }
  //   } catch (Exception e) {

  //     e.printStackTrace();
  //     Debug("Logger: writeToNewLogFile has Failed!");

  //   }
  // }

  /**
   * Writes the log into a file
   * 
   * @param log
   */
  private static void writeToLogFile(String log) {

    try {

      File logFile238 = new File("/home/lvuser/logFile238.txt");

      // If the file already exists, open the file and write the string to it
      if (logFile238.exists()) {

        FileWriter logFile = new FileWriter("/home/lvuser/logFile238.txt", true);
        logFile.write(newline + log);
        logFile.flush();
        logFile.close();

        // If the file doesn't already exists, create a new one and write the string to
        // it
      } else {

        logFile238.createNewFile();
        FileWriter logFile = new FileWriter("/home/lvuser/logFile238.txt", true);
        logFile.write(newline + log);
        logFile.flush();
        logFile.close();

      }
    } catch (Exception e) {

      e.printStackTrace();
      Debug("Logger: writeToLogFile has Failed!");

    }
  }

  /*
   * public static void writeToUSB(String log,String logFileName){ // /U or
   * media/svb? try{
   * 
   * File customFile = new File("/U"+logFileName+".txt");
   * 
   * //If the file already exists, open the file and write the string to it
   * if(customFile.exists()) {
   * 
   * FileWriter logFile = new FileWriter("/U"+logFileName+".txt",true);
   * logFile.write(newline+log); logFile.flush(); logFile.close();
   * 
   * //If the file doesn't already exists, create a new one and write the string
   * to it } else {
   * 
   * customFile.createNewFile(); FileWriter logFile = new
   * FileWriter("/U"+logFileName+".txt",true); logFile.write(newline+log);
   * logFile.flush(); logFile.close();
   * 
   * } } catch(Exception e) {
   * 
   * e.printStackTrace(); Log("Logger: writeToNewLogFile has Failed!");
   * 
   * } }
   */

  public enum Level {
    Trace(0), Debug(1), Info(2), Warn(3), Error(4),;

    private int value;
    private static HashMap<Integer, Level> map = new HashMap<>();

    private Level(int level) {
      this.value = level;
    }

    static {
      for (Level level : Level.values()) {
        map.put(level.value, level);
      }
    }

    public static Level valueOf(int level) {
      return (Level) map.get(level);
    }

    public int getValue() {
      return value;
    }
  }
}