function Out = CheckDistVec3(P1,P2,dist)
    if norm( osimVec3ToArray(P1) - osimVec3ToArray(P2) ) > dist
        Out = true;
    else
        Out = false;
    end
end