package org.xero1425.base.subsystems.vision;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.xero1425.base.subsystems.Subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystem extends Subsystem {

    public class Retro {
        public Translation2d pts[] ;
        public Pose3d camToTarget ;
        public Pose3d robotToField ;
        public Pose3d robotToTarget ;
        public Pose3d targetToCamera ;
        public Pose3d targetToRobot ;
        public double ta ;
        public double tx ;
        public double txp ;
        public double ty ;
        public double typ ;
    } ;

    public class Fiducial {
        public int id ;
        public String family ;
        public Pose3d camToTarget ;
        public Pose3d robotToField ;
        public Pose3d robotToTarget ;
        public Pose3d targetToCamera ;
        public Pose3d targetToRobot ;
        public double ta ;
        public double tx ;
        public double txp ;
        public double ty ;
        public double typ ;
    } ;

    public class Detector {
        public String className ;
        public int classId ;
        public double confidence ;
        public Translation2d [] pts ;
        public double ta ;
        public double tx ;
        public double txp ;
        public double ty ;
        public double typ ;
    } ;

    public class Classifier {
        public String className ;
        public int classId ;
        public double confidence ;
    }

    private boolean found_ ;
    private int id_ ;
    private double tl_ ;
    private double ts_ ;
    private boolean valid_targets_ ;

    private Retro [] retro_ ;
    private Fiducial[] fuds_ ;
    private Detector[] detectors_ ;
    private Classifier[] classifiers_ ;

    public LimeLightSubsystem(Subsystem parent, String name) {
        super(parent, name) ;
    }

    public int getId() {
        return id_ ;
    }

    public double getTL() {
        return tl_ ;
    }

    public double getTS() {
        return ts_ ;
    }

    public boolean validTargets() {
        return valid_targets_ ;
    }

    public boolean isLimelightFound() {
        return found_ ;
    }

    public Retro[] getRetroData() {
        return retro_ ;
    }

    public Fiducial[] getFiducialData() {
        return fuds_ ;
    }

    public Detector[] getDetectorData() {
        return detectors_ ;
    }

    public Classifier[] getClassifierData() {
        return classifiers_ ;
    }

    @Override
    public void computeState() {
        String json = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("") ;

        if (json.length() == 0) {
            found_ = false ;
        }
        else {
            Object obj = JSONValue.parse(json);
            if (obj instanceof JSONObject) {
                parseLimelightJsonObject((JSONObject)obj) ;
            }
        }
    }

    private String getStringFromObject(JSONObject obj, String name, String def) {
        String ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof String) {
                ret = (String)temp ;
            }
        }

        return ret ;
    }

    private double getDoubleFromObject(JSONObject obj, String name, double def) {
        double ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof Double) {
                ret = (Double)temp ;
            }
        }

        return ret ;
    }

    private int getIntFromObject(JSONObject obj, String name, int def) {
        int ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof Double) {
                double d = (Double)temp ;
                ret = (int)d ;
            }
            else if (temp instanceof Integer) {
                ret = (Integer)temp ;
            }
        }

        return ret ;
    }

    private Pose3d getPose3dFromObject(JSONObject obj, String name, Pose3d def) {
        Pose3d ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof double[]) {
                double [] data = (double [])temp ;
                if (data.length == 6) {
                    Translation3d trans = new Translation3d(data[0], data[1], data[2]) ;
                    Rotation3d rot = new Rotation3d(data[3], data[4], data[5]) ;
                    ret = new Pose3d(trans, rot) ;
                }
            }       
        } 

        return ret ;
    }

    private Translation2d[] getPointArrayFromObject(JSONObject obj, String name, Translation2d[] def) {
        Translation2d[] ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof double[]) {
                double [] data = (double [])temp ;
                if (data.length % 2 == 0) {
                    ret = new Translation2d[data.length / 2] ;
                    for(int i = 0 ;i < data.length ; i += 2) {
                        ret[i / 2] = new Translation2d(data[i], data[i + 1]) ;
                    }
                }
            }       
        } 

        return ret ;
    }

    private void parseClassifier(JSONArray entries) {
        if (entries.size() == 0) {
            classifiers_ = null  ;
        }
        else {
            classifiers_ = new Classifier[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject cobj = (JSONObject)temp ;
                Classifier c = new Classifier() ;

                c.className = getStringFromObject(cobj, "class", "") ;
                c.classId = getIntFromObject(cobj, "classID", -1) ;
                c.confidence = getDoubleFromObject(cobj, "conf", 0.0) ;

                classifiers_[i] = c ;
            }
        }
    }

    private void parseDetector(JSONArray entries) {
        if (entries.size() == 0) {
            detectors_ = null  ;
        }
        else {
            detectors_ = new Detector[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject dobj = (JSONObject)temp ;
                Detector d = new Detector() ;
    
                d.className = getStringFromObject(dobj, "class", "") ;
                d.classId = getIntFromObject(dobj, "classID", -1) ;
                d.confidence = getDoubleFromObject(dobj, "conf", 0.0) ;
                d.pts = getPointArrayFromObject(dobj, "pts", null) ;
                d.ta = getDoubleFromObject(dobj, "ta", 0.0) ;
                d.tx = getDoubleFromObject(dobj, "tx", 0.0) ;
                d.txp = getDoubleFromObject(dobj, "txp", 0.0) ;
                d.ty = getDoubleFromObject(dobj, "ty", 0.0) ;
                d.typ = getDoubleFromObject(dobj, "typ", 0.0) ;

                detectors_[i] = d ;
            }
        }
    }

    private void parseFiducials(JSONArray entries) {
        if (entries.size() == 0) {
            fuds_ = null ;
        }
        else {
            fuds_ = new Fiducial[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject fud = (JSONObject)temp ;
                Fiducial f = new Fiducial() ;
                f.id = getIntFromObject(fud, "fID", -1) ;
                f.family = getStringFromObject(fud, "fam", "") ;
                f.camToTarget = getPose3dFromObject(fud, "t6c_ts", null) ;
                f.robotToField = getPose3dFromObject(fud, "t6r_fs", null) ;
                f.robotToTarget = getPose3dFromObject(fud, "t6r_ts", null) ;
                f.targetToCamera = getPose3dFromObject(fud, "t6t_cs", null) ;
                f.targetToRobot = getPose3dFromObject(fud, "t6t_rs", null) ;
                f.ta = getDoubleFromObject(fud, "ta", 0.0) ;
                f.tx = getDoubleFromObject(fud, "tx", 0.0) ;
                f.txp = getDoubleFromObject(fud, "txp", 0.0) ;
                f.ty = getDoubleFromObject(fud, "ty", 0.0) ;
                f.typ = getDoubleFromObject(fud, "typ", 0.0) ;

                fuds_[i] = f ;
            }
        }
    }

    private void parseRetro(JSONArray entries) {
        if (entries.size() == 0) {
            retro_ = null ;
        }
        else {
            retro_ = new Retro[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject robj = (JSONObject)temp ;
                Retro r = new Retro() ;

                r.pts = getPointArrayFromObject(robj, "pts", null) ;
                r.camToTarget = getPose3dFromObject(robj, "t6c_ts", null) ;
                r.robotToField = getPose3dFromObject(robj, "t6r_fs", null) ;
                r.robotToTarget = getPose3dFromObject(robj, "t6r_ts", null) ;
                r.targetToCamera = getPose3dFromObject(robj, "t6t_cs", null) ;
                r.targetToRobot = getPose3dFromObject(robj, "t6t_rs", null) ;
                r.ta = getDoubleFromObject(robj, "ta", 0.0) ;
                r.tx = getDoubleFromObject(robj, "tx", 0.0) ;
                r.txp = getDoubleFromObject(robj, "txp", 0.0) ;
                r.ty = getDoubleFromObject(robj, "ty", 0.0) ;
                r.typ = getDoubleFromObject(robj, "typ", 0.0) ;

                retro_[i] = r ;
            }
        }
    }

    private void parseLimelightJsonObject(JSONObject obj) {
        Object temp ;

        id_ = getIntFromObject(obj, "pID", 0) ;
        tl_ = getDoubleFromObject(obj, "tl", 10000.0) ;
        ts_ = getDoubleFromObject(obj, "ts", 0.0) ;
        double v = getDoubleFromObject(obj, "v", 0.0) ;
        if (Math.abs(v) < 0.001) {
            valid_targets_ = false ;
        }
        else {
            valid_targets_ = true ;
        }
    
        temp = obj.get("Classifier") ;
        if (temp instanceof JSONArray) {
            parseClassifier((JSONArray)temp) ;
        }
        else {
            classifiers_ = null ;
        }

        temp = obj.get("Detector") ;
        if (temp instanceof JSONArray) {
            parseDetector((JSONArray)temp) ;
        }
        else {
            detectors_ = null ;
        }

        temp = obj.get("Fiducial") ;
        if (temp instanceof JSONArray) {
            parseFiducials((JSONArray)temp);
        }        
        else {
            fuds_ = null ;
        }

        temp = obj.get("Retro") ;
        if (temp instanceof JSONArray) {
            parseRetro((JSONArray)temp) ;
        }
        else {
            retro_ = null ;
        }
    }
}
