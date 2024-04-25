using UnityEngine;

public class GeneratedObject : MonoBehaviour 
{
    private void Awake()
    {
        // Ensure that objects which are touching an out of bounds terrain piece are
        // deleted, to prevent weird visual glitches.

        if(TryGetComponent<Collider>(out var collider))
        {
            var other = Physics.OverlapBox(collider.bounds.center, collider.bounds.extents);

            foreach(var otherc in other)
            {
                if(otherc.TryGetComponent<OOBTerrainMarker>(out _))
                {
                    Destroy(gameObject);
                    return;
                }
            }
        }
    }
}