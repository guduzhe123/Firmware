// This is an example Custom Command Qml file. You have full access to the entire Qml language
// for creating any user interface you like. From the ui you can affect the following changes
// with respect to your vehicle:
//    1) Sending COMMAND_LONG commands out over mavlink using QGCButton control
//    2) Modifying parameters
//
// When developing custom Qml file implementations. You must restart QGroundControl to pick up
// the changes. You need to do this even if you select Clear Qml file. Not sure what at the this
// point. Qt must be caching the files somewhere.

import QtQuick 2.2

//debug 等待50秒自动关闭23电机
/* static int timer = 0;
if(timer++ > 400*50){
    if(i == 2 || i == 3){
        outputs[i] = -1;
    }
} */

import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FactControls 1.0
import QGroundControl.Controllers 1.0

FactPanel {
    id: panel

    property var qgcView: null // Temporary hack for broken QGC parameter validation implementation

    CustomCommandWidgetController { id: controller; factPanel: panel }

    // Your own custom changes start here - everything else above is always required

    Column {
        // The QGCButton control is provided by QGroundControl.Controls. It is a wrapper around
        // the standard Qml Button element which using the default QGC font and color palette.

        QGCButton {
            text: "Set Home to current position"
            // Arguments to CustomCommandWidgetController::sendCommand (Mavlink COMMAND_LONG)
            //   command id
            //   component id
            //   confirmadtion
            //   param 1-7
            onClicked: controller.sendCommand(179, 0, 0, 1, 0, 0, 0, 0, 0, 0)
        }

        /* QGCButton {
            text: "MAV_CMD_DO_MOUNT_CONTROL"
            // Arguments to CustomCommandWidgetController::sendCommand (Mavlink COMMAND_LONG) MNT_MAV_COMPID, 67
            //   command id
            //   component id
            //   confirmadtion
            //   param 1-7
            onClicked: controller.sendCommand(205, 0, 0, 10, 0, 50, 0, 0, 0, 0)
        }

        QGCButton {
            text: "MAV_CMD_DO_MOUNT_CONFIGURE"
            // Arguments to CustomCommandWidgetController::sendCommand (Mavlink COMMAND_LONG)
            //   command id
            //   component id
            //   confirmadtion
            //   param 1-7
            onClicked: controller.sendCommand(204, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "MAVLINK_MSG_ID_LANDING_TARGET"
            // Arguments to CustomCommandWidgetController::sendCommand (Mavlink COMMAND_LONG) MNT_MAV_COMPID, 67
            //   command id
            //   component id
            //   confirmadtion
            //   param 1-7
            onClicked: controller.sendCommand(205, 0, 0, 10, 0, 50, 0, 0, 0, 0)
        } */

        //Command to trigger a motor stop
        //|action (0= disable(not stop), 1=enable(stop))| stop number (we use 123456, >>012345)| single or a pair(0 = 1 motor, 1 = a pair)| Empty| Empty| Empty| Empty|

        QGCButton {
            text: "STOP NO.1"
            onClicked: controller.sendCommand(701, 0, 0, 1, 1, 0, 0, 0, 0, 0)
        }
        QGCButton {
            text: "STOP NO.2"
            onClicked: controller.sendCommand(701, 0, 0, 1, 2, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "STOP NO.3"
            // Arguments to CustomCommandWidgetController::sendCommand (Mavlink COMMAND_LONG)
            //   command id
            //   component id
            //   confirmadtion
            //   param 1-7
            onClicked: controller.sendCommand(701, 0, 0, 1, 3, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "STOP NO.4"
            onClicked: controller.sendCommand(701, 0, 0, 1, 4, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "STOP NO.3&NO.4"
            onClicked: controller.sendCommand(701, 0, 0, 1, 3, 1, 0, 0, 0, 0)
        }

        QGCButton {
            text: "STOP NO.5"
            onClicked: controller.sendCommand(701, 0, 0, 1, 5, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "STOP NO.6"
            onClicked: controller.sendCommand(701, 0, 0, 1, 6, 0, 0, 0, 0, 0)
        }



        QGCButton {
            text: "Re enable all motors"
            onClicked: controller.sendCommand(701, 0, 0, 0, 4, 0, 0, 0, 0, 0)
        }

        // The FactTextField control is provides by GroundControl.FactControls. It is a wrapper
        // around the Qml TextField element which allows you to bind it directly to any parameter.
        // The parameter is changed automatically when you click enter or click away from the field.
        // Understand that there is currently no value validation. So you may crash your vehicle by
        // setting a parameter to an incorrect value. Validation will come in the future.

        // Be very careful when referencing parameters. If you specify a parameter which does not exist
        // QGroundControl will warn and shutdown.

        /* FactTextField {
            // The -1 signals default component id.
            // You can replace it with a specific component id if you like
            fact: controller.getParameterFact(-1, "MAV_SYS_ID")
        } */
    }

    // Your own custom changes end here - everything else below is always required
}
