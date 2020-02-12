local path = minetest.get_modpath"modelsynth"
local datastructures = dofile(path .. "/datastructures.lua")

local function get_label_key(data)
	return data[1] .. "," .. data[2] .. "," .. data[3]
	--~ return table.concat(data, ",")
end

local function vector_index(x, y, z, ystride, zstride)
	return z * zstride + y * ystride + x
end

-- label2 should be offset by +X, +Y or +Z from label1
local function can_neighbour(adjacencies, label1, label2, num_labels)
	return adjacencies[num_labels * label1 + label2]
end

local function generate_modelinfo(pos1, pos2)
	-- Labels are model pieces, indices start from zero
	local labels = {}
	local label_indices = {}
	local num_labels = 0

	local vm = minetest.get_voxel_manip()
	local e1, e2 = vm:read_from_map(pos1, pos2)
	local area = VoxelArea:new{MinEdge=e1, MaxEdge=e2}
	local nodes = vm:get_data()

	-- The 1x3x1 nodes sized labels in the areas
	local label_maps = {{}, {}, {}}

	-- Collect all labels and their positions
	for y_off = 0,2 do
		-- One label map for each offset in the y direction
		local label_map = label_maps[y_off+1]
		local y_start = pos1.y + y_off
		-- Go back two nodes in y direction so that the labels are searched
		-- only in the area between pos1 and pos2
		local y_end = pos2.y - 2
		local li = 0
		for z = pos1.z, pos2.z do
			for y = y_start, y_end, 3 do
				for x = pos1.x, pos2.x do
					-- The label consists of 1x3x1 nodes
					local vi_bottom = area:index(x, y, z)
					local vi_middle = vi + area.ystride
					local vi_top = vi_middle + area.ystride
					local label = {nodes[vi_bottom], nodes[vi_middle],
						nodes[vi_top]}
					-- Add the label if it isn't known yet
					local k = get_label_key(label)
					local i = label_indices[k]
					if not i then
						-- Add this newly found label
						i = num_labels
						label_indices[k] = i
						labels[i] = label
						num_labels = num_labels+1
					end
					label_map[li] = i
					li = li+1
				end
			end
		end
	end

	-- Find neighbouring constraints
	local adjacencies = {x = {}, y = {}, z = {}}
	for y_off = 0,2 do
		local label_map = label_maps[y_off+1]
		local y_start = pos1.y + y_off
		local y_end = pos2.y - 2
		local label_ystride = pos2.x - pos1.x + 1
		-- The number of times the for loop along y was executed previously
		local label_zstride = math.floor((y_end - y_start) / 3) + 1
		-- One step along z means label_ystride along y
		label_zstride = label_zstride * label_ystride
		local li = 0
		for z = pos1.z, pos2.z-1 do
			for y = y_start, y_end-3, 3 do
				for x = pos1.x, pos2.x-1 do
					-- Only +x, +y, +z transitions are saved because of symmetry
					local label_current = label_map[li]
					local label_z = label_map[li + label_zstride]
					local label_y = label_map[li + label_ystride]
					local label_x = label_map[li + 1]
					adjacency_i = label_current * num_labels
					adjacencies.z[adjacency_i + label_z] = true
					adjacencies.y[adjacency_i + label_y] = true
					adjacencies.x[adjacency_i + label_x] = true
					li = li+1
				end
				-- One x is skipped
				li = li + 1
			end
			-- One slice along x is skipped due to the the for loop end
			li = li + label_ystride
		end
	end

	-- Save node names so that the nodes in the labels are not ephemeral
	local id_to_nodename = {}
	for k = 1,num_labels do
		local label = labels[k]
		for j = 1,3 do
			local nodeid = label[j]
			id_to_nodename[nodeid] = d_to_nodename[nodeid]
				or minetest.get_name_from_content_id(nodeid)
		end
	end

	-- Find the empty space and other special labels, this code is not yet
	-- flexible
	-- They are used for the intial model and boundary conditions
	local airid = minetest.get_name_from_content_id"air"
	local k = get_label_key{airid, airid, airid}
	local air_label_id = label_indices[k]
	if not air_label_id then
		minetest.log("warning", "No air empty space label found")
	end
	special_labels = {empty_space = air_label_id}

	return {
		num_labels = num_labels,
		labels = labels,
		nodeids = nodeids,
		adjacencies = adjacencies,
		special_labels = special_labels,
	}
end

-- Removes invalid labels from a single catalog entry,
-- returns false if nothing has changed
local function remove_invalid_labels(ci_prev, ci, test_adjacency, num_labels,
		catalog)
	local have_changes = false
	for label = 0, num_labels-1 do
		if catalog[ci * num_labels + label] then
			-- Test if there is a label at ci_prev which allows `label` to be
			-- at ci
			local label_valid = false
			for label_prev = 0, num_labels-1 do
				if catalog[ci_prev * num_labels + label_prev] then
					if test_adjacency(label_prev, label) then
						label_valid = true
						break
					end
				end
			end
			-- Remove the label from the catalog if it is not allowed
			if not label_valid then
				catalog[ci * num_labels + label] = nil
				have_changes = true
			end
		end
	end
	return have_changes
end

-- An implementation of Arc Consistency
local function apply_neighbour_constraints(constraints, catalog_strides,
		num_labels, catalog, arcs_stack)
	-- The strides represent the direction, e.g. +X, -X
	local strides = {catalog_strides.x, -catalog_strides.x, catalog_strides.y,
		-catalog_strides.y, catalog_strides.z, -catalog_strides.z}
	-- The adjacency matrices for the various strides
	local stride_to_adjacencies = {
		[catalog_strides.x] = constraints.x,
		[-catalog_strides.x] = constraints.x,
		[catalog_strides.y] = constraints.y,
		[-catalog_strides.y] = constraints.y,
		[catalog_strides.z] = constraints.z,
		[-catalog_strides.z] = constraints.z
	}
	-- FIXME: Maybe a FIFO queue works better here
	while not arcs_stack:is_empty() do
		local arc = arcs_stack:pop()
		local ci_prev = arc[1]
		local stride = arc[2]
		local ci = ci_prev + stride
		local adjacencies = stride_to_adjacencies[stride]
		local test_adjacency
		if stride < 0 then
			-- Negative direction
			function test_adjacency(label1, label2)
				return can_neighbour(adjacencies, label2, label1, num_labels)
			end
		else
			function test_adjacency(label1, label2)
				return can_neighbour(adjacencies, label1, label2, num_labels)
			end
		end
		if remove_invalid_labels(ci_prev, ci, test_adjacency, num_labels,
				catalog) then
			-- FIXME: this resembles Flood Fill, there may be a faster algorithm
			-- Labels were removed, so continue with neighbours of ci
			for i = 1,#strides do
				local next_stride = strides[i]
				if next_stride ~= stride then
					arcs_stack:push{ci, next_stride}
				end
			end
		end
	end
end

local function get_index_function(ystride, zstride)
	return function(x, y, z)
		return z * zstride + y * ystride + x
	end
end

-- Returns false if it fails
local function generate_chunk(modelinfo, index_model, region, model)
	-- Start and end positions in the region
	local p1 = region[1]
	local p2 = region[2]
	-- Save the current model in the region for the failure case
	local model_backup = {}
	for z = p1.z, p2.z do
		for y = p1.y, p2.y do
			local mi = index_model(p1[1], y, z)
			for x = p1.x, p2.x do
				model_backup[#model_backup+1] = model[mi]
				mi = mi+1
			end
		end
	end
	-- The catalog needs to be 1 bigger than the region in all directions
	local wc = p2[1] - p1[1] + 1 + 2
	local hc = p2[2] - p1[2] + 1 + 2
	local lc = p2[3] - p1[3] + 1 + 2
	local function index_catalog(x, y, z, label)
		local ci = vector_index(x, y, z, wc, hc * wc)
		return ci * num_labels + label
	end
	local function index_model_off(cx, cy, cz)
		return index_model(p1[1]-1 + cx, p1[2]-1 + cy, p1[3]-1 + cz)
	end
	-- Initialize catalog
	local num_labels = modelinfo.num_labels
	local catalog = {}
	-- Add all labels to all positions in the catalog
	for i = 0, wc * hc * lc - 1 do
		for label = 0, num_labels-1 do
			catalog[i * num_labels + label] = true
		end
	end
	-- Remove labels from the border, where the model is fixed
	-- FIXME: this can be done more efficiently
	for cz = 0, lc do
		for cy = 0, hc do
			for cx = 0, wc do
				if (cz == 0 or cz == lc-1) or (cy == 0 or cy == hc-1)
						or (cx == 0 or cx == wc-1) then
					local fixed_label = model[index_model_off(cx, cy, cz)]
					for label = 0, num_labels-1 do
						if label ~= fixed_label then
							catalog[index_catalog(cx, cy, cz, label)] = nil
						end
					end
				end
			end
		end
	end
	-- Apply AC3 as preprocessing to remove invalid labels from the catalog
	constraints = modelinfo.adjacencies
	local catalog_strides = {x = 1, y = wc, z = hc * wc}
	local arcs = {}
	local num_arcs = 0
	-- Arcs to the border of the catalog don't need to be added
	for cz = 1, lc - 2 do
		for cy = 1, hc - 2 do
			for cx = 1, wc - 2 do
				-- Add arcs from all possible neighbours
				local ci = vector_index(x, y, z, wc, hc * wc)
				arcs[num_arcs+1] = {ci - catalog_strides.x, catalog_strides.x}
				arcs[num_arcs+2] = {ci + catalog_strides.x, -catalog_strides.x}
				arcs[num_arcs+3] = {ci - catalog_strides.y, catalog_strides.y}
				arcs[num_arcs+4] = {ci + catalog_strides.y, -catalog_strides.y}
				arcs[num_arcs+5] = {ci - catalog_strides.z, catalog_strides.z}
				arcs[num_arcs+6] = {ci + catalog_strides.z, -catalog_strides.z}
				num_arcs = num_arcs + 6
			end
		end
	end
	local arcs_stack = datastructures.create_stack{input = arcs}
	-- Here the catalog should have at least one label for each position
	apply_neighbour_constraints(constraints, catalog_strides, num_labels,
		catalog, arcs_stack)

	-- Do the Discrete Model Synthesis in the region
	for z = 1, lc-2 do
		for y = 1, hc-2 do
			-- Index for the model
			local mi = index_model_off(1, y, z)
			-- Index for the catalog
			local ci = index_catalog(1, y, z)
			for x = 1, wc-2 do
				local possible_labels = {}
				for label = 0, num_labels-1 do
					if catalog[ci * num_labels + label] then
						possible_labels[#possible_labels+1] = label
					end
				end
				if #possible_labels == 0 then
					-- It failed, so reset the model to its backup
					local backup_i = 1
					for z = p1.z, p2.z do
						for y = p1.y, p2.y do
							local mi = index_model(p1[1], y, z)
							for x = p1.x, p2.x do
								model[mi] = model_backup[backup_i]
								backup_i = backup_i+1
								mi = mi+1
							end
						end
					end
					return false
				end

				local chosen_label = possible_labels[
					math.random(#possible_labels)]
				model[mi] = chosen_label
				for i = 1, #possible_labels do
					local label = possible_labels[i]
					if label ~= chosen_label then
						catalog[ci * num_labels + label] = nil
					end
				end

				-- Remove labels which became invalid because a label has been
				-- chosen for position ci
				local strides = {catalog_strides.x, -catalog_strides.x,
					catalog_strides.y, -catalog_strides.y, catalog_strides.z,
					-catalog_strides.z}
				for i = 1,#strides do
					arcs_stack:push{ci, strides[i]}
				end
				apply_neighbour_constraints(constraints, catalog_strides,
					num_labels, catalog, arcs_stack)

				mi = mi+1
				ci = ci+1
			end
		end
	end

	return true
end

local function generate_model(pos1, pos2, modelinfo)
	local w = pos2.x - pos1.x
	local h = pos2.y - pos1.y
	if h % 3 > 0 then
		minetest.log("warning", "Height is not a multiple of 3")
		h = h - h % 3
	end
	local l = pos2.z - pos1.z

	-- Generate the model
	-- Use bigger width and height for the boundary conditions
	local wo = w + 2
	local ho = h + 2
	local lo = l + 2
	local model = {}
	-- Fill everything with empty space
	local empty_space = modelinfo.special_labels.empty_space
	for i = 0, wo * ho * lo - 1 do
		model[i] = empty_space
	end
	-- Subdivide the to-be-generated model into overlapping smaller regions
	local regions = datastructures.create_queue()
	local region_size_max = 2 * 16
	for z = 1, lo-2, region_size_max / 2 do
		for y = 1, ho-2, region_size_max / 2 do
			for x = 1, wo-2, region_size_max / 2 do
				local p1 = {x, y, z}
				local p2 = {
					math.max(x + region_size_max-1, wo-2),
					math.max(y + region_size_max-1, ho-2),
					math.max(z + region_size_max-1, lo-2),
				}
				regions:add{p1, p2}
			end
		end
	end
	-- Function which returns an index in the model array
	local index_model = get_index_function(wo, ho * wo)
	-- Generate in each region, add more regions if it failed
	repeat
		local region = regions:take()
		if not generate_chunk(modelinfo, index_model, region, model) then
			-- It failed, so subdivide the region
			local p1 = region[1]
			local p2 = region[2]
			local region_sizes = {}
			-- 3*3*3 new regions
			-- TODO

		end
	until regions.is_empty()
end

-- TODO: save generated model, use worldedit positions, add model synthesis,
-- chatcommands to use this mod
