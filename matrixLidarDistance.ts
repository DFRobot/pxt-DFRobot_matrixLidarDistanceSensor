let CMD_SETMODE                 = 1
let CMD_ALLData                 = 2
let CMD_FIXED_POINT             = 3
let CMD_LINE                    = 4
let CMD_LIST                    = 5
let CMD_AVOID_OBSTACLE          = 6
let CMD_CONFIG_AVOID            = 7
let CMD_OBSTACLE_DISTANCE       = 8
let CMD_END                     = CMD_OBSTACLE_DISTANCE
let STATUS_SUCCESS              = 0x53   
let STATUS_FAILED               = 0x63  
let IIC_MAX_TRANSFER            = 32     
let I2C_ACHE_MAX_LEN            = 32
let DEBUG_TIMEOUT_MS            = 8000
let ERR_CODE_NONE               = 0x00 
let ERR_CODE_CMD_INVAILED       = 0x01 
let ERR_CODE_RES_PKT            = 0x02 
let ERR_CODE_M_NO_SPACE         = 0x03 
let ERR_CODE_RES_TIMEOUT        = 0x04 
let ERR_CODE_CMD_PKT            = 0x05 
let ERR_CODE_SLAVE_BREAK        = 0x06 
let ERR_CODE_ARGS               = 0x07 
let ERR_CODE_SKU                = 0x08  
let ERR_CODE_S_NO_SPACE         = 0x09 
let ERR_CODE_I2C_ADRESS         = 0x0A 
let _addr = 0x30
let outDir = 0
let outEmergencyFlag = 0


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

    export enum Addr{
        //% block="0x30"
        Addr1 = 0x30,
        //% block="0x31"
        Addr2 = 0x31,
        //% block="0x32"
        Addr3 = 0x32,
        //% block="0x33"
        Addr4 = 0x33,
    }
    export enum Matrix {
        //% block="4 X 4"
        X4 = 1,
        //% block="8 X 8"
        X8 = 2,
    }


    /**
     * Initialize Matrix LiDAR Distance sensor
     */
    //% block="Initialize Matrix LiDAR Distance sensor I2C address $address"
    //% address.defl=Addr.Addr1
    //% weight=97
    export function initialize(address: Addr):void{}

    /**
     * Get obstacle avoidance data
     */
    //% block="Get obstacle avoidance data"
    //% weight=95
    export function getData():void{
        let length = 0
        let sendBuffer = pins.createBuffer(4);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_AVOID_OBSTACLE
        pins.i2cWriteBuffer(_addr, sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(CMD_AVOID_OBSTACLE)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS) {
            outDir = buf[4]
            outEmergencyFlag = buf[5]
        }
    }
    
    /**
     * For the setting of obstacle avoidance distance, the sensor will make suggestions for obstacle avoidance based on the distance value of detecting obstacles in front of it.
     * @param distance to distance ,eg: 10
     */

    //% block="Set obstacle avoidance distance $distance"
    //% distance.min=10 distance.max=50 distance.defl=10
    //% weight=90
    export function setObstacleDistance(distance: number):void{
        let length = 2
        let _wall = distance * 10
        let sendBuffer = pins.createBuffer(6);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_CONFIG_AVOID
        sendBuffer[4] = (_wall >> 8) & 0xff
        sendBuffer[5] = _wall & 0xff
        pins.i2cWriteBuffer(_addr,sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(CMD_CONFIG_AVOID)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS){
            let len = buf[2] << 8 | buf[3]
        }
    }

    /**
     * Check if the detected distance value has reached the critical distance for an imminent collision, which is a fixed value of 10cm. If this distance is detected, issue a warning.
     */

    //% block="Get hazard warning"
    //% weight=80
    export function emergencyWarning(): number {
        return outEmergencyFlag
    }

    /**
     * The API will provide steering suggestions for the car based on the data detected in front of the vehicle.
    */

    //% block="Get suggested direction"
    //% weight=70
    export function obstacleSuggestion(): number {
        return outDir
    }

    /**
     * By the system's comprehensive calculation, the distance values of obstacles in the current left, center, and right directions are detected in real time.
     * @param side to side ,eg: ObstacleSide.Left
    */

    //% block="Distance detection %side"
    //% weight=60
    export function getObstacleDistance(side: ObstacleSide): number {
        let length = 0
        let ret = 0
        let sendBuffer = pins.createBuffer(4);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_OBSTACLE_DISTANCE
        pins.i2cWriteBuffer(_addr, sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(CMD_OBSTACLE_DISTANCE)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS) {
            switch (side){
                case ObstacleSide.Left:
                    ret = buf[4] | buf[5] << 8
                    break
                case ObstacleSide.Front:
                    ret = buf[6] | buf[7] << 8
                    break
                case ObstacleSide.Right:
                    ret = buf[8] | buf[9] << 8
                    break
                default:
                    break

            }
        }
        return ret
        
    }

    /**
     * Specify one of the point output data, which is used to simplify the difficulty of project development and reduce the amount of data.
     * @param x to x ,eg: 3
     * @param y to y ,eg: 3
    */

    //% block="Specified point x: %x y: %y"
    //% weight=50
    //% x.min=0 x.max=7 x.defl=3
    //% y.min=0 y.max=7 y.defl=3
    export function matrixPointOutput(x: number, y: number): number {
        let length = 2
        let ret = 0
        let sendBuffer = pins.createBuffer(6);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_FIXED_POINT
        sendBuffer[4] = x
        sendBuffer[5] = y
        pins.i2cWriteBuffer(_addr, sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(CMD_FIXED_POINT)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS) {
            ret = buf[4] | buf[5] << 8
        }
        return ret
    }

    function recvPacket(cmd: number): Buffer{
        let sendBuffer = pins.createBuffer(10);
        let time = control.millis()
        while (control.millis() - time < DEBUG_TIMEOUT_MS)
        {
            let status = pins.i2cReadNumber(_addr, NumberFormat.Int8LE)
            if (status != 0xff){
                if (status == STATUS_SUCCESS || status == STATUS_FAILED){
                    sendBuffer[0] = status
                    let command = pins.i2cReadNumber(_addr, NumberFormat.Int8LE)
                    sendBuffer[1] = command
                    //serial.writeValue("cmd", command)
                    if (command != cmd){
                        return sendBuffer
                    }
                    let lenBuf = pins.i2cReadBuffer(_addr,2)
                    let len  = lenBuf[1] << 8 | lenBuf[0]
                    //serial.writeValue("len", len)
                    sendBuffer[2] = lenBuf[0]
                    sendBuffer[3] = lenBuf[1]
                    if(len == 0){
                        return sendBuffer
                    }
                    let dataBuf = pins.i2cReadBuffer(_addr, len)
                    for(let i = 0;i < len;i++){
                        sendBuffer[4+i] = dataBuf[i]
                    }
                    return sendBuffer

                }
            }
        }
        return sendBuffer
    }
}

