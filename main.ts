matrixLidarDistance.setAddr(48, 4)
basic.forever(function () {
    serial.writeLine("" + (matrixLidarDistance.matrixPointOutput(3, 3)))
    basic.pause(100)
})
