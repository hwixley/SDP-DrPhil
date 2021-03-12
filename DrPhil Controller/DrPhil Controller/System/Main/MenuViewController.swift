//
//  MenuViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 27/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit
import FirebaseAuth

class MenuViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
    }
    
    //MARK: Actions
    @IBAction func clickLogout(_ sender: UIButton) {
        let ac = UIAlertController(title: "Log Out", message: "Are you sure you want to log out?", preferredStyle: UIAlertController.Style.alert)
        ac.addAction(UIAlertAction(title: "Cancel", style: UIAlertAction.Style.cancel, handler: nil))
        
        ac.addAction(UIAlertAction(title: "Log Out", style: UIAlertAction.Style.default, handler: { (action) in
            do {
                try Auth.auth().signOut()
            } catch {
                self.navigationItem.prompt = "Log out error: please try again"
                return
            }
            self.tabBarController?.navigationController?.setNavigationBarHidden(true, animated: true)
            self.performSegue(withIdentifier: "logout", sender: self)
        }))
        
        if #available(iOS 13.0, *) {
            ac.view.tintColor = UIColor.systemPink
        }
        
        self.present(ac, animated: true, completion: nil)
    }
    
    @IBAction func unwindToMenuSegue(_ segue: UIStoryboardSegue) {
    }
}
