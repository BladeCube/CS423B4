using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class CameraController : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
          if (Input.GetButtonDown("Fire2")){
            setDestination();
          }
    }

    void setDestination(){
      RaycastHit temp;
      Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

      if(!Physics.Raycast(ray, out temp)){
        return;
      }

      AgentManager.destination = temp.point;
      AgentManager.SetAgentDestinations(temp.point);

    }

}
