classdef TicTacToeBoard
    % Class that handles a Tic Tac Toe Game Board
    %   Detailed explanation goes here
    
    properties (Constant)
        % This constants specify the values each square of the board can
        % take, and who's the winner for the players
        EMPTY = 0;
        PLAYER_X = 1;
        PLAYER_O = 2;
        
        % This constant specifies the value for when the game ends in a
        % draw
        DRAW = 3;
        
        % This map variable is only useful when pritting to the console
        STR_MAP =  [' ' 'X' 'O'];
        
        % Function that 
        SCORES = containers.Map([TicTacToeBoard.PLAYER_X, TicTacToeBoard.EMPTY, ...
                                 TicTacToeBoard.PLAYER_O], [1 0 -1]);
    end
    
    properties
        boardSize       % Length of one size of the board
        board           % Matrix that holds the board state
        currentPlayer = TicTacToeBoard.PLAYER_X; % Variable to identify whose turn is
    end
    
    methods
        
        % Class constructor. It can receive the board size and a matrix
        % stating the current board tokens positions.
        %
        % The default values if either boardSize or board are not given are
        % boardSize = 3 with an empty board.
        function self = TicTacToeBoard(boardSize, board)
            if nargin < 1
                self.boardSize = 3;
                self.board(1:self.boardSize, 1:self.boardSize) = self.EMPTY;
            elseif nargin < 2
                self.boardSize = boardSize;
                self.board(1:self.boardSize, 1:self.boardSize) = self.EMPTY;
            else
                self.boardSize = boardSize;
                self.board = board;
            end
        end
        
        % Board size getter function
        function size = getSize(self)
            size = self.boardSize;
        end
        
        % Function to retrieve the value of a square given its row and col
        % coordinates.
        function sqrValue = getSqrValue(self, row, col)
            sqrValue = self.board(row, col); 
        end
        
        % Returns the player whose turn is next
        function player = getCurrentPlayer(self)
            player = self.currentPlayer;
        end
        
        % Checks if the square at row, col is empty
        function empty = isSqrEmpty(self, row, col)
            empty = self.board(row, col) == self.EMPTY;
        end
        
        % Returns a nx2 matrix which contains the row, col values of all
        % empty squares
        function emptySqrs = getEmptySqrs(self)
            emptySqrs = [];
            for row = 1:self.boardSize
                for col = 1:self.boardSize
                    if self.board(row, col) == self.EMPTY
                        emptySqrs = [emptySqrs; row col];
                    end
                end
            end
        end
        
        % board getter function
        function board = getBoard(self)
            board = self.board;
        end
        
        % Function that erases the current board state to play a new one
        function self = resetBoard(self)
            self.board(1:self.boardSize, 1:self.boardSize) = self.EMPTY;
        end
        
        % Function that changes the player who is next to play
        function self = changePlayer(self)
            if self.currentPlayer == self.PLAYER_X
                self.currentPlayer = self.PLAYER_O;
            else
                self.currentPlayer = self.PLAYER_X;
            end
        end
        
        % Function that makes the current player to place a token in the
        % square given by row, col if the square is empty
        function self = move(self, row, col)
            if self.board(row, col) == self.EMPTY
                self.board(row, col) = self.currentPlayer;
            end
        end
        
        % Function that returns PLAYER_X or PLAYER_O if one of them won,
        % DRAW if it was a draw and EMPTY if the game hasn't finish yet.
        function winner = checkWinner(self)            
            % Get the horizontal lines
            lines = self.board;
            
            % Get the vertical lines
            for col = 1:self.boardSize
               lines = [lines; self.board(:, col)']; 
            end
            
            % Get the diagonals
            for i = 1:self.boardSize
                diag1(i) = self.board(i,i);
                diag2(i) = self.board(i, self.boardSize - i + 1);
            end
            
            lines = [lines; diag1; diag2];
            
            % Check if a line has all its values equal and different than
            % EMPTY
            for i = 1:size(lines, 1)
                line = lines(i, :);
                
                if line(1) ~= self.EMPTY && all(line == line(1))
                    winner = line(1);
                    return;
                end
            end
            
            % If there are no empty squares is a draw or the game is still
            % in progress otherwise.
            if size(self.getEmptySqrs(), 1) == 0
                winner = self.DRAW;
            else
                winner = self.EMPTY;
            end
        end
    end
    
end

