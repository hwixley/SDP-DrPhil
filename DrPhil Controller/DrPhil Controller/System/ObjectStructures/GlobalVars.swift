//
//  GlobalVars.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 30/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import Foundation
import UIKit

struct UserInfo {
    static var schedule : CleanSchedule? = nil
    static var dayStats : DailyStats? = nil
    static var resources : ResourceMetrics? = nil
    static var status : String? = nil
    static var map : UIImage? = nil
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
