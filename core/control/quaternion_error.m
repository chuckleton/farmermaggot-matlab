function deltaq = quaternion_error(q1,q2)
%QUATERNION_ERROR Computes delta q, the quaternion error from q1 to q2
deltaq = quatmultiply(quatinv(q1),q2);
end

