import openai
import torch
import pprint

class Hyperparameters:
    def __init__(self, MAX_STEPS=10, P=0.5, TEMP=0.3, WT_SCENE=0.25, MAX_EXAMPLES=10, SAMPLE_MATCH_NUM=1,
                 LLM_ACT=0.3, WT_ACT=1.0, STEP_CUTOFF_THRESHOLD=1.0, OPENAI_KEY=None, sampling_params=None):
        if torch.cuda.is_available():
            torch.cuda.set_device(0)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.MAX_STEPS = MAX_STEPS
        self.P = P  # Hyperparameter for early stopping heuristic to detect whether Planning LM believes the plan is finished
        self.TEMP = TEMP
        self.WT_SCENE = WT_SCENE
        self.MAX_EXAMPLES = MAX_EXAMPLES
        self.SAMPLE_MATCH_NUM = SAMPLE_MATCH_NUM
        self.LLM_ACT = LLM_ACT
        self.WT_ACT = WT_ACT
        self.STEP_CUTOFF_THRESHOLD = STEP_CUTOFF_THRESHOLD

        openai.api_key = OPENAI_KEY
        self.sampling_params = sampling_params or {
            "max_tokens": 1000,
            "temperature": 1.5,
            "n": 5,
            "logprobs": True,
            "presence_penalty": 0,
            "frequency_penalty": 0.3,
        }

        self.max_tokens = 10
        self.stop = '\n'

    def __repr__(self):
        params = {
            "MAX_STEPS": self.MAX_STEPS, "P": self.P, "TEMP": self.TEMP, "WT_SCENE": self.WT_SCENE,
            "MAX_EXAMPLES": self.MAX_EXAMPLES, "SAMPLE_MATCH_NUM": self.SAMPLE_MATCH_NUM, "LLM_ACT": self.LLM_ACT,
            "WT_ACT": self.WT_ACT, "STEP_CUTOFF_THRESHOLD": self.STEP_CUTOFF_THRESHOLD,
            "sampling_params": self.sampling_params
        }
        pprint.pprint(params)
        return ""

    def __str__(self):
        return self.__repr__()


class APVariables:
    def __init__(self):
        self.query_task = ""
        self.query_obj_names = []
        self.query_surface_names = []
        self.action_list = []
        self.condition_list = []
        self.feedback = ""
        self.action_plan = []
        self.scene_relationships = []

        self.all_actions = []
        self.action_list_embds = []
        self.prompt = ""
        self.explanation = ""
        self.previous_action = ""
        self.confirm_action = ""
        self.action_conditions = {}
        self.attached_object_name = ""
        self.step = 0