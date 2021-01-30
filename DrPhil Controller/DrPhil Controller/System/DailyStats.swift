//
//  DailyStats.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 30/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import Foundation

struct DailyStats {
    var round : Int? = nil
    var maxRound : Int? = nil
    var room : Int? = nil
    var maxRooms : Int? = nil
    var numHandles : Int? = nil
    var batteryUsage : Int? = nil
    var disinfectantUsage : Int? = nil
}

struct ResourceMetrics {
    var battery : Double? = nil
    var disinfectant : Double? = nil
}
