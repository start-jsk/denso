$(function() {
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":8888"
    });

    // class PendantStatus
    var PendantStatus = function(spec) {
        if (!spec) {
            spec = {};
        }
        this.button_names = ["key_manual",
                             "key_auto",
                             "key_teach_check",
                             "button_motor",
                             "button_lock",
                             "button_rsel",
                             "button_mmod",
                             "button_speed",
                             "button_stop",
                             "button_cancel",
                             "button_ok",
                             "button_shift",
                             "button_f1",
                             "button_f2",
                             "button_f3",
                             "button_f4",
                             "button_f5",
                             "button_f6",
                             "button_arrow_up",
                             "button_arrow_left",
                             "button_arrow_right",
                             "button_arrow_down",
                             "button_j1_plus",
                             "button_j1_minus",
                             "button_j2_plus",
                             "button_j2_minus",
                             "button_j3_plus",
                             "button_j3_minus",
                             "button_j4_plus",
                             "button_j4_minus",
                             "button_j5_plus",
                             "button_j5_minus",
                             "button_j6_plus",
                             "button_j6_minus",
                             "scroll_up",
                             "scroll_down",
                             "touch_x",
                             "touch_y",
                             "touched",
                            ];
        for (var i = 0; i < this.button_names.length; i++) {
            this[this.button_names[i]] = spec[this.button_names[i]];
        }
    };

    PendantStatus.prototype.diff = function(another) {
        var ret = [];
        for (var i = 0; i < this.button_names.length; i++) {
            if (this[this.button_names[i]] !== another[this.button_names[i]]) {
                ret.push([this.button_names[i], another[this.button_names[i]]]);
            }
        }
        return ret;
    };


    function updateStatusButton(active_button) {
        var buttons = ["#running-button", "#stop-button"];
        for (var i = 0; i < buttons.length; i++) {
            if (buttons[i].toString() == active_button.toString()) {
                $(buttons[i])
                    .removeClass("btn-default")
                    .removeClass("disabled")
                    .addClass("btn-primary");
            }
            else {
                $(buttons[i])
                    .addClass("disabled")
                    .removeClass("btn-primary")
                    .addClass("btn-default");
            }
        }
    };
    function updateModeButton(active_button) {
        var buttons = ["#auto-button", "#manual-button", "#teach-button"];
        for (var i = 0; i < buttons.length; i++) {
            if (buttons[i].toString() == active_button.toString()) {
                $(buttons[i])
                    .removeClass("btn-default")
                    .removeClass("disabled")
                    .addClass("btn-primary");
            }
            else {
                $(buttons[i])
                    .addClass("disabled")
                    .removeClass("btn-primary")
                    .addClass("btn-default");
            }
        }
    };
    // visualize robot
    var viewer = new ROS3D.Viewer({
        divID : 'urdf',
        width : $("#urdf").width(),
        height : $("#urdf").height(),
        antialias : true,
        cameraPose: {
            x: 1.0,
            y: 1.0,
            z: 1.0
        }
    });
    viewer.setCenter({x: -0.2, y: -0.2, z: 0});
    viewer.addObject(new ROS3D.Grid({size: 1, color: '#00ffff'}));
    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : "/BASE"
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
        ros : ros,
        tfClient : tfClient,
        param: 'robot_description',
        rootObject : viewer.scene
    });

    function dateFormat(date) {
        var now = new Date();
        var diff = (now - date) / 1000.0; // diff in sec
        if (diff < 60) {   // within sec
            return Math.ceil(diff) + " secs ago";
        }
        else if (diff < 60 * 60) { // within hour
            return Math.ceil(diff / 60) + " mins ago";
        }
        else if (diff < 24 * 60 * 60) { // within day
            return Math.ceil(diff / 60 / 60) + " hours ago";
        }
        else {
            return Math.ceil(diff / 24 / 60 / 60) + " days ago";
        }
    };

    function addMessage(msg, level) {
        var alerts = $("#message-region .alert");
        var latest_alerts = alerts.slice(0, 3);
        var new_message = $('<div class="alert alert-' + level + '">' + msg +
                            '<span class="time pull-right">now</span>' +
                            '</div>');
        new_message.find(".time").attr("data-date", new Date());
        $("#message-region .alert").remove();
        $("#message-region").prepend(new_message)
            .append(latest_alerts);
    };

    function updateTimestamp() {
        var $alerts = $("#message-region .alert");
        $.each($alerts, function(i, alert) {
            var $alert = $(alert);
            var date = new Date($alert.find(".time").attr("data-date"));
            $alert.find(".time").html(dateFormat(date));
        });
        setTimeout(updateTimestamp, 1000);
    };
    updateTimestamp();
    // subscribing denso pendant
    var sub = new ROSLIB.Topic({
        ros: ros,
        name: "/denso_pendant_publisher/status",
        messageType: "denso_pendant_publisher/PendantStatus"
    });
    var last_status = null;
    sub.subscribe(function(msg) {
        var current_status = new PendantStatus(msg);
        if (last_status) {
            var diff = last_status.diff(current_status);
            for (var i = 0; i < diff.length; i++) {
                var diff_element = diff[i];
                var diff_name = diff_element[0];
                var diff_val = diff_element[1];
                // Auto/Manual/Teach mode
                if ((diff_name.toString() === "key_auto".toString())
                    && diff_val === true) {
                    updateModeButton("#auto-button");
                }
                else if ((diff_name.toString() === "key_manual".toString())
                         && diff_val === true) {
                    updateModeButton("#manual-button");
                }
                else if ((diff_name.toString() === "key_teach_check".toString())
                         && diff_val === true) {
                    updateModeButton("#teach-button");
                }

                if (((diff_name.toString() === "button_stop".toString())
                     || (diff_name.toString() === "button_cancel".toString()) )
                    && diff_val === true) {
                    if (running_status) {
                        addMessage("will stop next time", "info");
                    }
                    else {
                            addMessage("already stopped", "danger");
                    }
                }
                else if ((diff_name.toString() === "button_ok".toString())
                    && diff_val === true) {
                    if (running_status) {
                        var now = new Date();
                        if (((now - running_status_updated_at) / 1000.0) > 1) {
                            addMessage("already started", "danger");
                        }
                    }
                    else {
                        addMessage("starting", "info");
                    }
                }
            }

            // emulate mouse movement using arrows
            var dtheta = 0.05;
            if (current_status.button_arrow_up) {
                viewer.cameraControls.rotateUp(dtheta);
            }
            if (current_status.button_arrow_down) {
                viewer.cameraControls.rotateUp(-dtheta);
            }
            if (current_status.button_arrow_left) {
                viewer.cameraControls.rotateLeft(dtheta);
            }
            if (current_status.button_arrow_right) {
                viewer.cameraControls.rotateLeft(-dtheta);
            }

        }
        else {
            if (current_status.key_auto) {
                updateModeButton("#auto-button");
            }
            else if (current_status.key_manual) {
                updateModeButton("#manual-button");
            }
            else if (current_status.key_teach_check) {
                updateModeButton("#teach-button");
            }
        }
        last_status = new PendantStatus(msg);
    });

    var running_status = false;
    var running_status_updated_at = null;
    var running_sub = new ROSLIB.Topic({
        ros: ros,
        name: "/irex_demo_running",
        messageType: "std_msgs/Bool"
    });
    running_sub.subscribe(function(msg) {
        if (msg.data) {         // true
            updateStatusButton("#running-button");
            if (!running_status) {
                addMessage("started", "success");
            }
        }
        else {
            updateStatusButton("#stop-button");
            if (running_status) {
                addMessage("stopped", "success");
            }
        }
        running_status = msg.data;
        running_status_updated_at = new Date();
    });

    // var info = new ROSLIB.Topic({
    //     ros: ros,
    //     name: "/rosout",
    //     messageType: "rosgraph_msgs/Log"
    // });
    // info.subscribe(function(msg) {
    //     if (msg.level === 2) {  // info
    //         var message = msg.msg;
    //         var prev_messages = $("#message-region p"); // max to 2
    //         // $("#message-region p").remove();
    //         // $("#message-region").prepend('<p class="rosinfo-message">' + message + '</p>')
    //         //     .append(prev_messages.slice(0, 2));
    //     }
    // });
});
