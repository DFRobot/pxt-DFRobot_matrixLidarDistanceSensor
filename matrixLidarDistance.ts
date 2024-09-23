


//% block="Matrix LiDAR Distance"
//% weight=100 color=#5b3fe8 icon="\uf0b2"
//% groups="['Distance measurement', 'Obstacle Avoidance']"
namespace matrixLidarDistance {

    export enum ObstacleSide {
        //% block="Left"
        Left = 1,
        //% block="Front"
        Front = 2,
        //% block="Right"
        Right = 3,
    }
    
    /**
     * For the setting of obstacle avoidance distance, the sensor will make suggestions for obstacle avoidance based on the distance value of detecting obstacles in front of it.
     * @param distance to distance ,eg: 10
     */

    //% block="Obstacle avoidance distance %distance"
    //% distance.min=10 distance.max=50 distance.defl=10
    //% weight=90
    export function setObstacleDistance(distance: number) {
        return 0
        
    }

    /**
     * Check if the detected distance value has reached the critical distance for an imminent collision, which is a fixed value of 10cm. If this distance is detected, issue a warning.
     */

    //% block="Emergency warning?"
    //% weight=80
    export function emergencyWarning(): boolean {
        return false;
    }

    /**
     * The API will provide steering suggestions for the car based on the data detected in front of the vehicle.
    */

    //% block="Suggestions for obstacle avoidance"
    //% weight=70
    export function obstacleSuggestion(): number {
        return 1;
    }

    /**
     * By the system's comprehensive calculation, the distance values of obstacles in the current left, center, and right directions are detected in real time.
     * @param side to side ,eg: ObstacleSide.Left
    */

    //% block="Distance detection %side"
    //% weight=60
    export function getObstacleDistance(side: ObstacleSide): number {
        return 1;
    }

    /**
     * Specify one of the point output data, which is used to simplify the difficulty of project development and reduce the amount of data.
     * @param x to x ,eg: 3
     * @param y to y ,eg: 3
    */

    //% block="matrix Output x %x y %y"
    //% weight=50
    //% x.min=0 x.max=7 x.defl=3
    //% y.min=0 y.max=7 y.defl=3
    export function matrixPointOutput(x: number, y: number): number {
        return 1;
    }

}

