/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.autonomous;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.MapperFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.core238.Logger;
import frc.robot.commands.IAutonomousCommand;

/**
 * Add your docs here.
 */
public class AutonomousModesReader {
    private final IAutonomousModeDataSource dataSource;

    public AutonomousModesReader(final IAutonomousModeDataSource dataSource) {
        this.dataSource = dataSource;
    }

    public HashMap<String, Command> getAutonmousModes() {
        final String classPath = "frc.robot.commands.";
        final HashMap<String, Command> autoModes = new HashMap<>();

        final List<AutonomousModeDescriptor> modeDescriptors = getAutonomousModeDescriptors();

        modeDescriptors.forEach(modeDescriptor -> {
            final String name = modeDescriptor.getName();
            final SequentialCommandGroup commands = new SequentialCommandGroup();
            final List<CommandGroupBase> parallelCommandGroups = new ArrayList<>();
            final List<Boolean> isParallelList = new ArrayList<>();
            isParallelList.add(false);
            parallelCommandGroups.add(new ParallelCommandGroup());

            modeDescriptor.getCommands().forEach(commandDescriptor -> {

                final String commandName = commandDescriptor.getName();
                final String className = classPath + commandName;
                // create a command object
                IAutonomousCommand autoCommand = null;
                try {
                    autoCommand = (IAutonomousCommand) Class.forName(className).getConstructor().newInstance();
                } catch (InstantiationException | IllegalAccessException | IllegalArgumentException
                        | InvocationTargetException | NoSuchMethodException | SecurityException
                        | ClassNotFoundException e) {
                    frc.core238.Logger
                            .Debug("AutonomousModesReader.getAutonmousModes unable to instantiate " + className);
                }
                String[] paramNames = {};
                try {
                     paramNames = Class.forName(className).getAnnotation(AutonomousModeAnnotation.class).parameterNames();
                } catch (ClassNotFoundException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                    
                }
                   
                
                Logger.Debug(commandDescriptor.getParameters().toString());
                
                // List<String> strippedParams =
                // commandDescriptor.getParameters().stream().skip(1).collect(Collectors.toList());
                // Value of first boolean parameter
                String parallelType = commandDescriptor.getParallelType();// Boolean.parseBoolean(commandDescriptor.getParameters().get(0));

                // Pass in parameters (minus isParallel
                autoCommand.setParameters(commandDescriptor.getParameters());
                autoCommand.setIsAutonomousMode(true);


                if (!parallelType.equals("None")) {
                    switch (parallelType) {
                        case "Parallel":
                            CommandGroupBase parallelGroup;
                            if (isParallelList.get(0) == false) { // If there is no parallel command group currently
                                                                  // being built
                                parallelGroup = new ParallelCommandGroup(); // Create a new one
                                parallelCommandGroups.set(0, parallelGroup); // Add the command
                                isParallelList.set(0, true); // Communicate that there IS a parallel command group being
                                                             // built
                            } else {
                                // Set parallelGroup to the last command group in the list
                                parallelGroup = parallelCommandGroups.get(parallelCommandGroups.size()-1);
                            }
                            if (autoCommand.getDelay() != 0) {
                                parallelGroup.addCommands(((Command)autoCommand).withTimeout(autoCommand.getDelay()));
                            } else {
                                parallelGroup.addCommands((Command) autoCommand); // Add the command in parallel
                            }
                            break;
                        case "Race":
                            CommandGroupBase raceGroup;
                            if (isParallelList.get(0) == false) { // If there is no parallel command group currently
                                                                  // being built
                                raceGroup = new ParallelRaceGroup(); // Create a new one
                                parallelCommandGroups.set(0, raceGroup); // Add the command
                                isParallelList.set(0, true); // Communicate that there IS a parallel command group being
                                                             // built
                            } else {
                                // Set raceGroup to the last command group in the list
                                raceGroup = parallelCommandGroups.get(parallelCommandGroups.size()-1);

                            }
                            if (autoCommand.getDelay() != 0) {
                                raceGroup.addCommands(((Command)autoCommand).withTimeout(autoCommand.getDelay()));
                            } else {
                                raceGroup.addCommands((Command) autoCommand); // Add the command in parallel
                            }
                            Logger.Debug("built race");
                            break;
                        case "Deadline_Leader":
                            CommandGroupBase deadlineGroup;
                            if (isParallelList.get(0) == false) { // If there is no parallel command group currently
                                                                  // being built
                                deadlineGroup = new ParallelDeadlineGroup((Command) autoCommand); // Create a new one
                                parallelCommandGroups.set(0, deadlineGroup); // Add the command
                                isParallelList.set(0, true); // Communicate that there IS a parallel command group being built
                            } else {
                                // Set deadlineGroup to the last command group in the list
                                deadlineGroup = parallelCommandGroups.get(parallelCommandGroups.size()-1);
                            }

                            if (autoCommand.getDelay() != 0) {
                                ((ParallelDeadlineGroup)deadlineGroup).setDeadline(((Command)autoCommand).withTimeout(autoCommand.getDelay()));
                            } else {
                                ((ParallelDeadlineGroup) deadlineGroup).setDeadline((Command) autoCommand);
                            }
                            
                            break;
                        case "Deadline_Follower":
                            CommandGroupBase deadlineFollowerGroup;
                            if (isParallelList.get(0)) {
                                deadlineFollowerGroup = parallelCommandGroups.get(parallelCommandGroups.size()-1);
                            } else {
                                
                                deadlineFollowerGroup = new ParallelDeadlineGroup((Command) autoCommand); // Create a new one
                                parallelCommandGroups.set(0, deadlineFollowerGroup); // Add the command
                                isParallelList.set(0, true); // Communicate that there IS a parallel command group being
                            }
                            if (autoCommand.getDelay() != 0) {
                                deadlineFollowerGroup.addCommands(((Command)autoCommand).withTimeout(autoCommand.getDelay()));
                            } else {
                                deadlineFollowerGroup.addCommands((Command) autoCommand);
                            }
                            break;
                    }

                } else {
                    if (isParallelList.get(0)) { // If there is a parallel command group being built
                        // Add that parallel command group to the sequential command list
                        commands.addCommands((Command) parallelCommandGroups.get(0));
                    }
                    isParallelList.set(0, false); // Communicate that there is not a parallel command group being built
                    if (autoCommand.getDelay() != 0) {
                        
                        commands.addCommands(((Command) autoCommand).withTimeout(Double.parseDouble(commandDescriptor.getParameters().get(0)))); // Add the command sequentially
                    } else {
                        commands.addCommands((Command) autoCommand); // Add the command sequentially
                    }
                }
            });

            if (isParallelList.get(0)) { // If there is a parallel command group being built
                // Add that parallel command group to the sequential command list
                // This is done again here to ensure that if the final command is parallel, it's
                // still added properly
                commands.addCommands((Command) parallelCommandGroups.get(0));
            }
            // add to dictionary
            autoModes.put(name, commands);
        });

        return autoModes;
    }

    private List<AutonomousModeDescriptor> getAutonomousModeDescriptors() {

        List<AutonomousModeDescriptor> modeDescriptors = new ArrayList<AutonomousModeDescriptor>();
        final String json = dataSource.getJson();

        if (json == null) {
            return modeDescriptors;
        }

        final ObjectMapper mapper = new ObjectMapper();
        mapper.configure(MapperFeature.ACCEPT_CASE_INSENSITIVE_PROPERTIES, true);

        try {
            AutonomousModeDescriptors descriptor = mapper.readValue(json,
                    new TypeReference<AutonomousModeDescriptors>() {
                    });
            modeDescriptors = descriptor.getAutonomousModes();
        } catch (JsonMappingException e) {
            Logger.Error(e.getStackTrace().toString());
        } catch (JsonProcessingException e) {
            Logger.Error(e.getStackTrace().toString());
        }

        return modeDescriptors;
    }

}
