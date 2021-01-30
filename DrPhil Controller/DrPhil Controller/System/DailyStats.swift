//
//  DailyStats.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 30/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import Foundation

struct DailyStats {
    var round : Int
    var maxRound : Int
    var room : Int
    var maxRooms : Int
    var numHandles : Int
    var batteryUsage : Int
    var disinfectantUsage : Int
}

struct ResourceMetrics {
    var battery : Double
    var disinfectant : Double
}
