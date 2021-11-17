using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;
using System;

public class Agent : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;
    private List<Collider> agents = new List<Collider>();
    private List<Collider> walls = new List<Collider>();

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();

    void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
    {
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        } else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (false)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }

        if (false)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    #endregion

    #region Incomplete Functions

    //IM JUST GONNA USE vi_0 (the desired speed) = 5 for now, we can change later

    private Vector3 ComputeForce()
    {
        var force = Vector3.zero;
        force = CalculateGoalForce() + CalculateAgentForce() ;
        //force = CalculateGoalForce() + CalculateAgentForce() + CalculateWallForce();
        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
    }

    private Vector3 CalculateGoalForce()
    {
      if(path.Count > 0){
        Vector3 ei_0 = (path[0] - transform.position).normalized;
        Vector3 vi = rb.velocity;
        float vi_0  = 5;
        float mass = rb.mass;

        Vector3 goalForce = (((vi_0 * ei_0) - vi)/0.5f) * mass;

        return goalForce;
      }
      else{
        return Vector3.zero;
      }

    }

    private Vector3 CalculateAgentForce()
    {
        Vector3 agentForce = new Vector3(0.0f, 0.0f, 0.0f);
        float ri = radius; //radius of agents

        for(int i = 0; i < agents.Count; i++){
          //declaring constants, not too sure what they're supposed to be I think we choose
          //Debug.Log("I is "+ i+ " walls.Count is "+agents.Count);
          float Ai = 1;
          float Bi = 1;
          float k_constant = 5;
          float kappa = 5;

          //below is used to calculate repulsion force and non penetration force
          float dist = Vector3.Distance(agents[i].gameObject.transform.position, transform.position); //distance between the two agents
          Vector3 n_ij = (transform.position - agents[i].gameObject.transform.position)/dist; //normal vector
          float g = dist > 0 ? dist : 0; // g function, 0 if dist < 0, distance otherwise

          //below is to calculate sliding/friction force
          Vector3 t_ij = new Vector3(-n_ij.z, 0.0f, n_ij.x);
          //Debug.Log(agents[i].gameObject.name+" "+ rb.gameObject.name);
          //Debug.Log(agents[i].attachedRigidbody.velocity +" "+ rb.velocity);
          Vector3 delta_v = Vector3.Scale(agents[i].attachedRigidbody.velocity - rb.velocity, t_ij);
          Vector3 repulsionForce = Ai * (float)Math.Exp((ri - dist) / Bi)*n_ij;
          Vector3 penetrationForce = k_constant*g*(ri - dist)* n_ij;
          Vector3 repulsion_and_penetration = (repulsionForce + penetrationForce) ;
          Vector3 slidingForce = (kappa*g)*Vector3.Scale(delta_v, t_ij);
          Vector3 agentForceij = repulsion_and_penetration + slidingForce;

          //all agent forces together

          //Debug.Log(agentForceij);
          agentForce += agentForceij;
        }

        return agentForce;
    }

    private Vector3 CalculateWallForce()
    {
      Vector3 wallForce = new Vector3(0.0f, 0.0f, 0.0f);
      float ri = radius; //radius of agents

      for(int i = 0; i < walls.Count; i++){
        //declaring constants, not too sure what they're supposed to be I think we choose
        float Ai = 1;
        float Bi = 1;
        float k_constant = 5;
        float kappa = 5;

        //below is used to calculate repulsion force and non penetration force
        float dist = Vector3.Distance(walls[i].gameObject.transform.position, transform.position); //distance between the two agents
        Vector3 n_ij = (transform.position - walls[i].gameObject.transform.position)/dist; //normal vector
        float g = dist > 0 ? dist : 0; // g function, 0 if dist < 0, distance otherwise

        //below is to calculate sliding/friction force
        Vector3 t_ij = new Vector3(-n_ij.z, 0.0f, n_ij.x);
        //Vector3 delta_v = Vector3.Scale(walls[i].attachedRigidbody.velocity - rb.velocity, t_ij);


        //all agent forces together

        Vector3 repulsionForce = Ai * (float)Math.Exp((ri - dist) / Bi)*n_ij;
        Vector3 penetrationForce = k_constant*g*(ri - dist)* n_ij;
        Vector3 repulsion_and_penetration = (repulsionForce + penetrationForce) ;
        Vector3 slidingForce = (kappa*g)*Vector3.Scale(Vector3.Scale(rb.velocity, t_ij),t_ij);
        Vector3 wallForceij = repulsion_and_penetration + slidingForce;
        Debug.Log(wallForceij);
        wallForce += wallForceij;
      }

      return wallForce;
    }

    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
      //when colliding with another agent, add to agent list
      if(other.gameObject.tag == "agent"){
        agents.Add(other);
      }
      if(other.gameObject.tag == "wall"){
        walls.Add(other);
      }
    }

    public void OnTriggerExit(Collider other)
    {
      //once agent leaves the collision radius, remove from list
      if(other.gameObject.tag == "agent"){
        agents.Remove(other);
      }
      if(other.gameObject.tag == "wall"){
        walls.Remove(other);
      }
    }

    public void OnCollisionEnter(Collision collision)
    {

    }

    public void OnCollisionExit(Collision collision)
    {

    }

    #endregion
}
