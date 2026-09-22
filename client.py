import requests
import API
invoke_url = "https://optimize.api.nvidia.com/v1/nvidia/cuopt"
fetch_url_format = "https://optimize.api.nvidia.com/v1/status/"

API = API.API()
headers = {
    "Authorization": f"Bearer {API}",
    "Accept": "application/json",
}

payload = {
  "action": "cuOpt_OptimizedRouting",
  "data": {
    "cost_waypoint_graph_data": None,
    "travel_time_waypoint_graph_data": None,
    "cost_matrix_data": {
      "data": {
        "1": [
          [
            0,
            1,
            1
          ],
          [
            1,
            0,
            1
          ],
          [
            1,
            1,
            0
          ]
        ],
        "2": [
          [
            0,
            1,
            1
          ],
          [
            1,
            0,
            1
          ],
          [
            1,
            2,
            0
          ]
        ]
      }
    },
    "travel_time_matrix_data": {
      "data": {
        "1": [
          [
            0,
            1,
            1
          ],
          [
            1,
            0,
            1
          ],
          [
            1,
            1,
            0
          ]
        ],
        "2": [
          [
            0,
            1,
            1
          ],
          [
            1,
            0,
            1
          ],
          [
            1,
            2,
            0
          ]
        ]
      }
    },
    "fleet_data": {
      "vehicle_locations": [
        [
          0,
          0
        ],
        [
          0,
          0
        ]
      ],
      "vehicle_ids": [
        "veh-1",
        "veh-2"
      ],
      "capacities": [
        [
          2,
          2
        ],
        [
          4,
          1
        ]
      ],
      "vehicle_time_windows": [
        [
          0,
          10
        ],
        [
          0,
          10
        ]
      ],
      "vehicle_break_time_windows": [
        [
          [
            1,
            2
          ],
          [
            2,
            3
          ]
        ]
      ],
      "vehicle_break_durations": [
        [
          1,
          1
        ]
      ],
      "vehicle_break_locations": [
        0,
        1
      ],
      "vehicle_types": [
        1,
        2
      ],
      "vehicle_order_match": [
        {
          "order_ids": [
            0
          ],
          "vehicle_id": 0
        },
        {
          "order_ids": [
            1
          ],
          "vehicle_id": 1
        }
      ],
      "skip_first_trips": [
        True,
        False
      ],
      "drop_return_trips": [
        True,
        False
      ],
      "min_vehicles": 2,
      "vehicle_max_costs": [
        7,
        10
      ],
      "vehicle_max_times": [
        7,
        10
      ]
    },
    "task_data": {
      "task_locations": [
        1,
        2
      ],
      "task_ids": [
        "Task-A",
        "Task-B"
      ],
      "demand": [
        [
          1,
          1
        ],
        [
          3,
          1
        ]
      ],
      "task_time_windows": [
        [
          0,
          5
        ],
        [
          3,
          9
        ]
      ],
      "service_times": [
        0,
        0
      ],
      "order_vehicle_match": [
        {
          "order_id": 0,
          "vehicle_ids": [
            0
          ]
        },
        {
          "order_id": 1,
          "vehicle_ids": [
            1
          ]
        }
      ]
    },
    "solver_config": {
      "time_limit": 1,
      "objectives": {
        "cost": 1,
        "travel_time": 0,
        "variance_route_size": 0,
        "variance_route_service_time": 0,
        "prize": 0
      },
      "verbose_mode": False,
      "error_logging": True
    }
  },
  "client_version": ""
}

# re-use connections
session = requests.Session()

response = session.post(invoke_url, headers=headers, json=payload)

while response.status_code == 202:
    request_id = response.headers.get("NVCF-REQID")
    fetch_url = fetch_url_format + request_id
    response = session.get(fetch_url, headers=headers)

response.raise_for_status()
response_body = response.json()
print(response_body)
