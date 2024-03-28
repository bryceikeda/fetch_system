#!/usr/bin/env python3

example_tasks = ["Pour the water from the bottle into the glass", "Wave hi!", "Do a wave!",
                 "Move the mustard to the right table", "Move the cheezeit to the right table",
                 "Move the pringles to the right table", "Move the cheezeit to the left table",
                 "move the food to the left table", "move the snacks to the right table", 
                 "Pick the cup", "Place the cheezeit on the left table"]

example_object_names_list = [["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["glass", "pringles", "mustard", "bottle", "cheezeit"],
                             ["cup", "pringles", "cheezeit"], 
                             ["cup", "pringles", "cheezeit"]]

example_surfaces_list = [["table on the right", "table on the left"], ["table on the right", "table on the left"],
                         ["table on the right", "table on the left"], ["table on the right", "table on the left"],
                         ["table on the right", "table on the left"], ["table on the right", "table on the left"],
                         ["table on the right", "table on the left"], ["table on the right", "table on the left"],
                         ["table on the right", "table on the left"], ["table on the right", "table on the left"], 
                         ["table on the right", "table on the left"]]

example_aps = [["pick bottle", "pour in glass", "place on table on the right", "done"],
               ["wave at me", "done"],
               ["wave at me", "done"],
               ["pick mustard", "place on table on the right", "done"],
               ["pick cheezeit", "place on table on the right", "done"],
               ["pick pringles", "place on table on the right", "done"],
               ["pick cheezeit", "place on table on the left", "done"],
               ["pick pringles", "place on table on the left", "pick mustard", "place on table on the left", "pick cheezeit", "place on table on the left", "done"],
               ["pick pringles", "place on table on the right", "pick mustard", "place on table on the right", "pick cheezeit", "place on table on the right", "done"], 
               ["pick cup", "done"], 
               ["place on table on the left", "done"]]