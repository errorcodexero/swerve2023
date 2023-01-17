package frc.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SimMotorController;

public class ArmModel extends SimulationModel {
    private SimMotorController first_ ;
    private double first_degrees_per_second_per_volt_ ;
    private double first_angle_ ;

    private SimMotorController second_ ;
    private double second_degrees_per_second_per_volt_ ;
    private double second_angle_ ;

    public ArmModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create() {
        first_ = new SimMotorController(this, "first") ;
        if (!first_.createMotor()) {
            return false ;
        }

        try {
            first_degrees_per_second_per_volt_ = getProperty("first:degrees_per_second_per_volt").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("first:degrees_per_second_per_volt").endMessage();
            return false ;
        }

        second_ = new SimMotorController(this, "second") ;
        if (!second_.createMotor())
            return false ;

        try {
            second_degrees_per_second_per_volt_ = getProperty("second:degrees_per_second_per_volt").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("second:degrees_per_second_per_volt").endMessage();
            return false ;
        }

        setCreated(); 
        return true ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        double power, encoder ;



        power = first_.getPower() ;
        first_angle_ += power * first_degrees_per_second_per_volt_ * dt ;
        encoder = first_angle_ / 360.0 * 42.0 ;
        first_.setEncoder(encoder);

        if (Math.abs(power) > 0.05) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("first ").add("angle", first_angle_).add("encoder", encoder).add("dt", dt).add("dps", first_degrees_per_second_per_volt_).endMessage();
        }

        power = second_.getPower() ;
        second_angle_ += power * second_degrees_per_second_per_volt_ * dt ;
        encoder = second_angle_ / 360.0 * 42.0 ;
        second_.setEncoder(encoder);
    }
}
