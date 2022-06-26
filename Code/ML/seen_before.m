function [Total_seen_landmarks, had_it_been_seen_before, N_real, mul] = seen_before(Total_seen_landmarks, SeenLandmarks, mul)

for i=1:length(SeenLandmarks)

    if Total_seen_landmarks(SeenLandmarks(i)) == 0

        had_it_been_seen_before(SeenLandmarks(i)) = false;
        Total_seen_landmarks(SeenLandmarks(i)) = 1;
        mul = [mul; SeenLandmarks(i)];

    else
        
        had_it_been_seen_before(SeenLandmarks(i)) = true;

    end



end

N_real = sum(Total_seen_landmarks);

end

