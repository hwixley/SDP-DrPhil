//
//  GlobalVars.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 30/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import Foundation
import UIKit

struct MyUser {
    static var robot: Robot? = nil
    static var statusInfo: StatusInfo? = nil
    static var tasks: [Task]? = []
}

struct Robot {
    var UID: String
    var robotID: String
    var schedule: CleanSchedule? = nil
}

struct StatusInfo {
    var status : Int
    var dayStats : DailyStats? = nil
    var resources : ResourceMetrics? = nil
    var map : UIImage? = nil
    
    func getStatus() -> String {
        if status == 0 {
            return "idle at charging station"
        } else if status == 1 {
            return "cleaning"
        } else if status == 2 {
            return "returning to charging station"
        } else if status == 3 {
            return "Dr Phil is stuck"
        } else if status == 4 {
            return "emergency halt procedure"
        }
        return ""
    }
}

struct Task {
    var task: Int
    var executionTime: String
    var date: String
    var taskID: Int
    
    func getTask() -> String {
        if task == 0 {
            return "Clean"
        } else if task == 1 {
            return "Return to charging station"
        }
        return "Stay idle at charging station"
    }
}

func isWeekday() -> Bool {
    let currentDate = Date()
    let df = DateFormatter()
    df.locale = Locale(identifier: "en_GB")
    df.dateFormat = "ccc"
    
    let day = df.string(from: currentDate)
    if day == "Sat" || day == "Sun" {
        return false
    } else {
        return true
    }
}
