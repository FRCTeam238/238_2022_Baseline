/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Add your docs here.
 */
public class BaseDashboard {

    public void init() {
        Logger.Trace("BaseDashboard.init() Start");
    }

    public <T> SimpleWidget buildElement(WidgetOptions<T> options) {
        ShuffleboardTab tab;
        if (options._tabName != null){
            tab = Shuffleboard.getTab(options._tabName);
        } else {
            tab = Shuffleboard.getTab(null);
        }

        SimpleWidget theWidget = tab.add(options._elementName, options._defaultValue);
        //testSweetEntries.put(elementName, theWidget.getEntry());
        theWidget
            .withSize(options._width, options._height)
            .withPosition(options._x, options._y);

        return theWidget;
    }

    public <T> ComplexWidget buildElement(WidgetOptions<T> options, Sendable sendable) {
        ShuffleboardTab tab;
        if (options._tabName != null){
            tab = Shuffleboard.getTab(options._tabName);
        } else {
            tab = Shuffleboard.getTab(null);
        }

        ComplexWidget theWidget = tab.add(options._elementName, sendable);
        //testSweetEntries.put(elementName, theWidget.getEntry());
        theWidget
            .withSize(options._width, options._height)
            .withPosition(options._x, options._y);

        return theWidget;
    }

    



    public class WidgetOptions<T> {
        private Integer _x = 0;
        private Integer _y = 0;
        private Integer _height = 0;
        private Integer _width = 0;
        private String _elementName;
        private String _tabName;
        private T _defaultValue;

    }

    public class WidgetOptionsBuilder<T> {
        private final WidgetOptions<T> widgetOptions;

        public WidgetOptionsBuilder(){
            widgetOptions = new WidgetOptions<T>();
        }

        public WidgetOptionsBuilder<T> x(Integer value){
            widgetOptions._x = value;
            return this;
        }

        public WidgetOptionsBuilder<T> y(Integer value){
            widgetOptions._y = value;
            return this;
        }

        public WidgetOptionsBuilder<T> height(Integer value){
            widgetOptions._height = value;
            return this;
        }

        public WidgetOptionsBuilder<T> width(Integer value){
            widgetOptions._width = value;
            return this;
        }

        public WidgetOptionsBuilder<T> defaultValue(T value){
            widgetOptions._defaultValue = value;
            return this;
        }

        public WidgetOptionsBuilder<T> elementName(String value){
            widgetOptions._elementName = value;
            return this;
        }

        public WidgetOptionsBuilder<T> tabName(String value){
            widgetOptions._tabName = value;
            return this;
        }

        public WidgetOptions<T> toWidgetOptions(){
            return widgetOptions;
        }
    }
}
