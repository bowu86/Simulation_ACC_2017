function plotGraph(T, plotProbFields, colorFields, plotedges)


% Color constants
redColor = [1 0 0];
blueColor = [0 0 1];
yellowColor = [1 1 0];
greenColor = [0 1 0];

if nargin <= 3
    plotedges = true;
end
if nargin <= 2
    colorFields = {};
end
if nargin <= 1
    plotProbFields = {};
end
        
% Add info
%text(0,100,sprintf('t=%1.1f',t));
%text(50,100,sprintf('Output="%s"',mat2str(cell2mat(currentword))));

%% Plot nodes
if isempty(plotProbFields)
    figure
    
    % Plot Edges
    if plotedges
        for ii=1:T.state_no
            for jj=find(T.adj(ii,:))
                if jj==ii
                    disp('Cannot plot self-loop');
                else
                    plot([T.nodes(ii).position(1),T.nodes(jj).position(1)],...
                        [T.nodes(ii).position(2),T.nodes(jj).position(2)],'r','Linewidth',.5);
                    hold on
                end
            end
        end
    end
    
    % Plot States
    for ii=1:T.state_no
        plot(T.nodes(ii).position(1), T.nodes(ii).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
        hold on
    end

    axis([0, (T.numCol+1)*10, 0, (T.numRow+1)*10]);
    axis equal
    axis off
else
    for fieldind = 1:length(plotProbFields)
        figure
                        
        % Plot Edges
        if plotedges
            for ii=1:T.state_no
                for jj=find(T.adj(ii,:))
                    if jj==ii
                        disp('Cannot plot self-loop');
                    else
                        plot([T.nodes(ii).position(1),T.nodes(jj).position(1)],...
                            [T.nodes(ii).position(2),T.nodes(jj).position(2)],'r','Linewidth',.5);
                        hold on
                    end
                end
            end
        end
        
        % Set Color
        if isempty(colorFields)
            color1 = blueColor;
            color2 = redColor;
        else
            colors = colorFields{fieldind};
            color1 = colors(1,:);
            color2 = colors(2,:);
        end
        
        probField = arrayfun(@(x) x.(plotProbFields{fieldind}), T.nodes);
        colorM = [];
        for ii=1:T.state_no
            %     filledCircle(T.nodes(ii).position,2,1000, );
            alpha = probField(ii)/max(probField);
            plot(T.nodes(ii).position(1), T.nodes(ii).position(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', alpha*color1 +  (1-alpha)*color2);
            colorM = [colorM; alpha*color1 +  (1-alpha)*color2];
            hold on
        end
        % plot colormap
        [~, sortI] = sort(probField);
        colorMSort = colorM;
        for jj = 1:size(colorM,1)
            colorMSort(jj,:) = colorM(sortI(jj),:);
        end
        colormap(colorMSort);
%         colorbar
        caxis([0, max(probField)]);
        
        axis([0, (T.numCol+1)*10, 0, (T.numRow+1)*10]);
        axis equal
        axis off
    end    
end
