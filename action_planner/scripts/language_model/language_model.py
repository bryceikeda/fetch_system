#!/usr/bin/env python3

import numpy as np
import torch
import string
from sentence_transformers import SentenceTransformer
from language_model.utils import *
from language_model.config import *
from language_model.dataset import *
import os

os.environ["TOKENIZERS_PARALLELISM"] = "false"


class LanguageModel:
    def __init__(self):
        if torch.cuda.is_available():
            torch.cuda.set_device(0)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.OPENAI_KEY = ""

        self.planning_lm_id = "gpt-3.5-turbo"
        self.translation_lm_id = "sentence-transformers/all-roberta-large-v1"

        self.generator = language_model_engine(self.planning_lm_id, self.OPENAI_KEY)
        self.translation_lm = SentenceTransformer(self.translation_lm_id).to(
            self.device
        )
        self.hyperparams = get_hyperparameters(OPENAI_KEY=self.OPENAI_KEY)

        self.example_task_embeddings = self.translation_lm.encode(
            example_tasks, batch_size=512, convert_to_tensor=True, device=self.device
        )

    def empty_cache(self):
        torch.cuda.empty_cache()

    def query_language_model(self, query):
        inst_AP_variables = APVariables()
        inst_AP_variables.query_task = query["task"]
        inst_AP_variables.query_obj_names = query["object_names"]
        inst_AP_variables.query_surface_names = query["surface_names"]
        inst_AP_variables.action_list = query["action_list"]
        inst_AP_variables.feedback = query["feedback"]
        inst_AP_variables.action_plan = query["action_plan"]
        inst_AP_variables.attached_object_name =  query["attached_object_name"]
        
        if query["mode"] == "setup trigger":
            inst_AP_variables.prompt = get_prompt(
                inst_AP_variables, None, "trigger explain"
            )  # construct prompt for explanation

            samples, log_probs = self.generator(
                inst_AP_variables.prompt,
                self.hyperparams.sampling_params,
                self.hyperparams.max_tokens,
                self.hyperparams.stop,
            )  # query Planning LM for action plan

            inst_AP_variables.explanation = samples[np.argmax(log_probs)]
            inst_AP_variables.prompt = get_prompt(
                inst_AP_variables, None, "trigger"
            )  # construct prompt

            samples, log_probs = self.generator(
                inst_AP_variables.prompt,
                self.hyperparams.sampling_params,
                self.hyperparams.max_tokens,
                self.hyperparams.stop,
            )

            trigger_str = samples[np.argmax(log_probs)].translate(
                str.maketrans("", "", string.punctuation)
            )
            print("Trigger:", trigger_str)

            if "no trigger" in trigger_str:
                query["mode"] = "generate action plan"
                return "no trigger", self.query_language_model(query)
            
            inst_AP_variables.prompt = get_prompt(
                inst_AP_variables, None, "extract command"
            ) # construct prompt
            
            samples, log_probs = self.generator(
                inst_AP_variables.prompt, 
                self.hyperparams.sampling_params, 
                self.hyperparams.max_tokens, 
                self.hyperparams.stop
            )
            
            command_str = samples[np.argmax(log_probs)].translate(str.maketrans('', '', string.punctuation))
            return trigger_str, command_str

        if query["mode"] == "generate action plan":
            # populate all actions
            for action in inst_AP_variables.action_list:
                if "<object>" in action:
                    for obj in inst_AP_variables.query_obj_names:
                        inst_AP_variables.all_actions.append(
                            action.replace("<object>", obj)
                        )
                elif "<surface>" in action:
                    for surface in inst_AP_variables.query_surface_names:
                        inst_AP_variables.all_actions.append(
                            action.replace("<surface>", surface)
                        )
                else:
                    inst_AP_variables.all_actions.append(action)
                inst_AP_variables.action_list_embds = self.translation_lm.encode(
                    inst_AP_variables.all_actions,
                    batch_size=512,
                    convert_to_tensor=True,
                    device=self.device,
                )

            # find most relevant example
            example_idx = get_similar_eg(
                inst_AP_variables.query_task,
                self.hyperparams,
                example_tasks,
                self.example_task_embeddings,
                self.translation_lm,
            )

            inst_AP_variables.prompt = get_prompt(
                inst_AP_variables, None, "explain"
            )  # construct prompt for explanation

            samples, log_probs = self.generator(
                inst_AP_variables.prompt,
                self.hyperparams.sampling_params,
                self.hyperparams.max_tokens,
                self.hyperparams.stop,
            )  # query Planning LM for single-step action candidates
            inst_AP_variables.explanation = samples[np.argmax(log_probs)]

            # for sample in samples:
            #   print("-------------------------", sample)
            # print(log_probs)
            # print(samples[np.argmax(log_probs)])

            check_AP = False
            while not check_AP:
                inst_AP_variables.prompt = get_prompt(
                    inst_AP_variables, example_idx, "action"
                ) # construct prompt
                
                generate_action_plan(
                    inst_AP_variables, self.generator, self.translation_lm, self.hyperparams
                ) # generate action plan
                
                inst_AP_variables.action_plan = [i for i in inst_AP_variables.action_plan if i != "action plan"]
                inst_AP_variables.feedback = ""
                print("action_plan:", inst_AP_variables.action_plan)
                
                check_AP, inst_AP_variables = check_action_plan(inst_AP_variables)
                print("AP feedback:", check_AP, "  ", inst_AP_variables.action_plan, "  ", inst_AP_variables.feedback)
                print("================================================================\n")

            return inst_AP_variables.action_plan


# if __name__ == "__main__":
#     language_model = LanguageModel()
#     # query = {
#     #     "mode": "generate action plan",
#     #     "task": "pour water to the glass and put it on the left table",
#     #     "object_names": ["glass", "pringles", "mustard", "bottle", "cheezeit"],
#     #     "surface_names": ["table on the right", "table on the left"],
#     #     "action_list": ["pick <object>", "place on <surface>", "pour in <object>", "wave at me", "done", "action plan"],
#     #     "feedback": "",
#     #     "action_plan": []}

#     # query = {
#     #     "mode": "setup trigger",
#     #     "task": "pour water to the glass and pick it up",
#     #     "object_names": ["glass", "pringles", "mustard", "bottle", "cheezeit"],
#     #     "surface_names": ["table on the right", "table on the left"],
#     #     "action_list": ["pick <object>", "place on <surface>", "pour in <object>", "wave at me", "done", "action plan"],
#     #     "feedback": "",
#     #     "action_plan": []}

#     # query = {
#     #     "mode": "setup trigger",
#     #     "task": "When the pringles is on the right table move bottle to the right table",
#     #     "object_names": ["glass", "pringles", "mustard", "bottle", "cheezeit"],
#     #     "surface_names": ["table on the right", "table on the left"],
#     #     "action_list": ["pick <object>", "place on <surface>", "pour in <object>", "wave at me", "done", "action plan"],
#     #     "feedback": "",
#     #     "action_plan": []}

#     query = {
#         "mode": "setup trigger",
#         "task": "Every 2 hours move pringles to the right table",
#         "object_names": ["glass", "pringles", "mustard", "bottle", "cheezeit"],
#         "surface_names": ["table on the right", "table on the left"],
#         "action_list": [
#             "pick <object>",
#             "place on <surface>",
#             "pour in <object>",
#             "wave at me",
#             "done",
#             "action plan",
#         ],
#         "feedback": "",
#         "action_plan": [],
#     }
#     print(language_model.query_language_model(query))
