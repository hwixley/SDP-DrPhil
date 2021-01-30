//
//  CleanSchedule.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 30/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import Foundation

struct CleanSchedule {
    
    var weekdays : TimeFrame? = nil
    var weekends : TimeFrame? = nil
}

struct TimeFrame {
    var start : String
    var end : String
    var numRounds : Int
}
