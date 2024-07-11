package frc.robot.subsystems;

public class ShootInfo {
    public double encoderValue;
    public double distance;
    public double upperPower;
    public double lowerPower;
    public ShootPosition type;
    public boolean auto;
    public double angle;
    public double intakeSpeed;
    public double rotationSpeed;
    public double finalIntakeSpeed;
    public double finalUpperPower;
    public double finalLowerPower;
    public double finalRotationSpeed;
    double[] angleValues = {50.59359786
       ,47.70163251
       ,46.14099054
       ,44.98544167
       ,43.9902074
       ,43.15865574
       ,42.43388811
       ,41.75527765
       ,41.15183708
       ,40.56902586
       ,40.05352506
       ,39.56336111
       ,39.09192643
       ,38.65650414
       ,38.23561229
       ,37.8401917
       ,37.46342451
       ,37.09613585
       ,36.74724062
       ,36.42056284
       ,36.10565647
       ,35.79239493
       ,35.50360444
       ,35.20511374
       ,34.9231826
       ,34.65655162
       ,34.4055305
       ,34.15525414
       ,33.91254556
       ,33.67228481
       ,33.44725727
       ,33.22096059
       ,33.01016543
       ,32.79901459
       ,32.59471956
       ,32.38570599
       ,32.19693031
       ,32.00407863
       ,31.81480674
       ,31.63744221
       ,31.46293402
       ,31.29471502
       ,31.12310751
       ,30.95454027
       ,30.79553392
       ,30.64156278
       ,30.49293888
       ,30.34418253
       ,30.20237973
       ,30.050031
       ,29.90919786
       ,29.7745695
       ,29.64666006
       ,29.52387699
       ,29.40048123
       ,29.271499
       ,29.14433203
       ,29.03184861
       ,28.91290387
       ,28.80397575
       ,28.68972398
       ,28.58829578
       ,28.48368971
       ,28.37867028
       ,28.27310338
       ,28.17862659
       ,28.07852813
       ,27.9898726
       ,27.89254942
       ,27.80955837
       ,27.71963079
       ,27.63791281
       ,27.55580532
       ,27.47332894
       ,27.38816478
       ,27.31416462
       ,27.24104556
       ,27.16429145
       ,27.08673993
       ,27.02675453
       ,26.94949632
       ,26.8870213
       ,26.82072456
       ,26.76346983
       ,26.69577744
       ,26.63956831
       ,26.58427002
       ,26.52646568
       ,26.47849615
       ,26.43154103
       ,26.37503044
       ,26.32048538
       ,26.2843977
       ,26.23798077
       ,26.18632043
       ,26.14410145
       ,26.11166028
       ,26.06758145
       ,26.03719872
       ,25.99046345
       ,25.9621819
       ,25.92841768
       ,25.89852514
       ,25.86532615
       ,25.83559564
       ,25.81097983
       ,25.78386771
       ,25.770402
       ,25.73930735
       ,25.71766056
       ,25.70462659
       ,25.68778188
       ,25.67714724
       ,25.65519903
       ,25.64950239
       ,25.63111518
       ,25.62422275
       ,25.61813015
       ,25.60490085
       ,25.59816156};
    double[] encoderValues = {0.497
        ,0.4977
        ,0.4984
        ,0.4991
        ,0.4998
        ,0.5005
        ,0.5012
        ,0.5019
        ,0.5026
        ,0.5033
        ,0.504
        ,0.5047
        ,0.5054
        ,0.5061
        ,0.5068
        ,0.5075
        ,0.5082
        ,0.5089
        ,0.5096
        ,0.5103
        ,0.511
        ,0.5117
        ,0.5124
        ,0.5131
        ,0.5138
        ,0.5145
        ,0.5152
        ,0.5159
        ,0.5166
        ,0.5173
        ,0.518
        ,0.5187
        ,0.5194
        ,0.5201
        ,0.5208
        ,0.5215
        ,0.5222
        ,0.5229
        ,0.5236
        ,0.5243
        ,0.525
        ,0.5257
        ,0.5264
        ,0.5271
        ,0.5278
        ,0.5285
        ,0.5292
        ,0.5299
        ,0.5306
        ,0.5313
        ,0.532
        ,0.5327
        ,0.5334
        ,0.5341
        ,0.5348
        ,0.5355
        ,0.5362
        ,0.5369
        ,0.5376
        ,0.5383
        ,0.539
        ,0.5397
        ,0.5404
        ,0.5411
        ,0.5418
        ,0.5425
        ,0.5432
        ,0.5439
        ,0.5446
        ,0.5453
        ,0.546
        ,0.5467
        ,0.5474
        ,0.5481
        ,0.5488
        ,0.5495
        ,0.5502
        ,0.5509
        ,0.5516
        ,0.5523
        ,0.553
        ,0.5537
        ,0.5544
        ,0.5551
        ,0.5558
        ,0.5565
        ,0.5572
        ,0.5579
        ,0.5586
        ,0.5593
        ,0.56
        ,0.5607
        ,0.5614
        ,0.5621
        ,0.5628
        ,0.5635
        ,0.5642
        ,0.5649
        ,0.5656
        ,0.5663
        ,0.567
        ,0.5677
        ,0.5684
        ,0.5691
        ,0.5698
        ,0.5705
        ,0.5712
        ,0.5719
        ,0.5726
        ,0.5733
        ,0.574
        ,0.5747
        ,0.5754
        ,0.5761
        ,0.5768
        ,0.5775
        ,0.5782
        ,0.5789
        ,0.5796
        ,0.5803};

    private boolean isNew = true;
    public ShootInfo(double encoderValue, double distance, double upperPower, double lowerPower, double intakeSpeed, double rotationSpeed, ShootPosition type) {
        this.distance = distance;
        this.upperPower = upperPower;
        finalUpperPower = upperPower;
        finalLowerPower = lowerPower;
        finalRotationSpeed = rotationSpeed;
        this.rotationSpeed = rotationSpeed;
        this.lowerPower = lowerPower;
        this.type = type;
        finalIntakeSpeed = intakeSpeed;
        this.intakeSpeed = 0;
        if(encoderValue != 0) {
            this.encoderValue = encoderValue;
        }
        else if(distance > 0) {
            this.encoderValue = getEncoderValue(distance);
        }
    }

    public double getEncoderValue(double distance) {
        
        angle = Math.asin((80/Math.sqrt(Math.pow(distance, 2) + Math.pow(80, 2))));
        
        int mid = angleValues.length/2;
        int left = 0;
        int right = angleValues.length-1;
        while(left < right) {

            if(angleValues[mid] < angle) {
                left = mid;
            }
            else {
                right = mid;
            }
            mid = left + (right - left) / 2;
        }
        encoderValue = encoderValues[left];
        System.out.println("ENCODER VALUE: " + encoderValue);
        return encoderValue;
    }

    public ShootInfo copy() {
        return new ShootInfo(encoderValue, distance, finalUpperPower, finalLowerPower, finalIntakeSpeed, finalRotationSpeed, type);
    }

    protected void setNew(boolean isNew) {
        this.isNew = isNew;
    }

    public boolean isNew() {
        return isNew;
    }

    public boolean isEqual(ShootInfo other) {
        return this.encoderValue == other.encoderValue && 
        this.distance == other.distance &&
        this.upperPower == other.upperPower &&
        this.lowerPower == other.lowerPower && 
        this.type.equals(other.type);
    }

}
